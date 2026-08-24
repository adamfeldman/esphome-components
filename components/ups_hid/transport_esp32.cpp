#include "transport_esp32.h"
#include "constants_ups.h"
#include "constants_hid.h"   // HID_REPORT_TYPE_* -- used by the descriptor-length parser
#include <cstdio>            // snprintf -- the usage-map self-proving log line
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"    // millis() -- retry spacing for the descriptor parse
#include "esphome/core/application.h"   // App.feed_wdt() -- see wait_for_transfer_()

#ifdef USE_ESP32

namespace esphome {
namespace ups_hid {

static const char *const ESP32_USB_TAG = "ups_hid.esp32_usb";

// USB HID Class defines
#ifndef USB_CLASS_HID
#define USB_CLASS_HID 0x03
#endif

Esp32UsbTransport::Esp32UsbTransport() {
    memset(&device_, 0, sizeof(device_));
}

Esp32UsbTransport::~Esp32UsbTransport() {
    deinitialize();
}

esp_err_t Esp32UsbTransport::initialize() {
    std::lock_guard<std::mutex> lock(device_mutex_);
    
    if (initialized_.load()) {
        return ESP_OK;
    }
    
    ESP_LOGI(ESP32_USB_TAG, "Initializing ESP32 USB transport");
    
    esp_err_t ret = setup_usb_host();
    if (ret != ESP_OK) {
        set_last_error("Failed to setup USB host: " + std::string(esp_err_to_name(ret)));
        return ret;
    }
    
    // Register USB client for device events - connection will be asynchronous  
    ret = find_and_open_device();
    if (ret != ESP_OK) {
        teardown_usb_host();
        return ret;
    }
    
    initialized_ = true;
    
    ESP_LOGI(ESP32_USB_TAG, "ESP32 USB transport initialized - waiting for USB device connection events");
    
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::deinitialize() {
    std::lock_guard<std::mutex> lock(device_mutex_);
    
    if (!initialized_.load()) {
        return ESP_OK;
    }
    
    ESP_LOGI(ESP32_USB_TAG, "Deinitializing ESP32 USB transport");
    
    connected_ = false;
    initialized_ = false;
    
    esp_err_t ret = teardown_usb_host();
    if (ret != ESP_OK) {
        ESP_LOGW(ESP32_USB_TAG, "USB teardown had issues: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

bool Esp32UsbTransport::is_connected() const {
    return connected_.load() && initialized_.load();
}

uint16_t Esp32UsbTransport::get_vendor_id() const {
    std::lock_guard<std::mutex> lock(device_mutex_);
    return device_.vendor_id;
}

uint16_t Esp32UsbTransport::get_product_id() const {
    std::lock_guard<std::mutex> lock(device_mutex_);
    return device_.product_id;
}

esp_err_t Esp32UsbTransport::hid_get_report(uint8_t report_type, uint8_t report_id, 
                                           uint8_t* data, size_t* data_len, 
                                           uint32_t timeout_ms) {
    if (!device_.dev_hdl) {
        ESP_LOGE(ESP32_USB_TAG, "HID GET_REPORT: No device handle");
        return ESP_ERR_INVALID_ARG;
    }
    if (!data || !data_len || *data_len == 0) {
        ESP_LOGE(ESP32_USB_TAG, "HID GET_REPORT: Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    if (leak_budget_exhausted_()) return ESP_ERR_INVALID_STATE;

    ESP_LOGD(ESP32_USB_TAG, "HID GET_REPORT: type=0x%02X, id=0x%02X, max_len=%zu", 
             report_type, report_id, *data_len);
    
    // Lazy, on the CALLER's task -- never on the USB client task (see the note
    // at connect: doing it there self-deadlocks). Retried, time-spaced.
    maybe_parse_report_descriptor_lengths_();

    uint8_t buffer[64] = {0};

    // ── wLength: the descriptor's declared length, NOT a fixed 64 ──────────
    // MEASURED 2026-08-22 on a CyberPower CP825LCD (0x0764:0x0501): asking for
    // 64 bytes of a report that is at most 6 bytes long is STALLed by that
    // device's firmware -- 0/23 declared reports answered at wLength 64, and
    // 23/23 answered at the declared length. Newer CP1500-generation firmware
    // tolerates the over-long ask and short-reads, which is the only reason
    // this defect stayed invisible. NUT has always sent the declared length
    // (drivers/libhid.c:131, rbuf->len[id] = replen[id] + 1) and exposes
    // over-long asks as an opt-IN knob (max_report_size), i.e. upstream treats
    // exact-size as the default and 64-style asks as the exception.
    //
    // FAIL-SAFE: when the descriptor is missing, unparseable, or simply does
    // not declare this report, fall back to the legacy fixed length. That is
    // byte-for-byte today's behaviour, so a parse failure degrades to the
    // status quo rather than breaking a device that works.
    size_t want = *data_len;
    if (report_lengths_known_ && report_type < 4) {
        uint8_t declared = report_payload_len_[report_type][report_id];
        if (declared > 0) {
            want = (size_t) declared + 1;   // + the leading report-id byte
            // One-shot POSITIVE confirmation that the fix is actually engaged,
            // deliberately DELAYED past the first poll. Everything the parse
            // logs happens inside the first update(), which cannot be captured
            // over the network -- so "working" and "silently inert" look
            // identical in any log you can take. Holding this line until a
            // later poll guarantees it is capturable. Without it the only
            // positive evidence is a DEBUG-level max_len that is not 64.
            if (!report_lengths_confirmed_logged_ &&
                (uint32_t)(millis() - report_lengths_known_at_ms_) >= REPORT_LENGTH_CONFIRM_DELAY_MS) {
                report_lengths_confirmed_logged_ = true;
                ESP_LOGI(ESP32_USB_TAG,
                         "Descriptor-derived report lengths are IN USE (e.g. type=0x%02X id=0x%02X "
                         "-> wLength %u, not the legacy %u).",
                         report_type, report_id, (unsigned) want, (unsigned) *data_len);
            }
        }
    }

    // Second delayed one-shot, same reason and same delay as above: the usage map
    // is built during the first poll, so this is the only place its summary can
    // actually be READ. Deliberately NOT nested inside the `declared > 0` branch
    // above -- an empty map is exactly the case worth hearing about.
    if (report_lengths_known_ && !usage_map_logged_ &&
        (uint32_t)(millis() - report_lengths_known_at_ms_) >= REPORT_LENGTH_CONFIRM_DELAY_MS) {
        usage_map_logged_ = true;
        char test_buf[28], freq_buf[28];
        if (probe_test_report_) snprintf(test_buf, sizeof(test_buf), "report 0x%02X", probe_test_report_);
        else                    snprintf(test_buf, sizeof(test_buf), "NOT FOUND -- map suspect");
        if (probe_freq_report_) snprintf(freq_buf, sizeof(freq_buf), "report 0x%02X", probe_freq_report_);
        else                    snprintf(freq_buf, sizeof(freq_buf), "not declared");
        ESP_LOGI(ESP32_USB_TAG,
                 "Usage map: %u usage(s), %u ambiguous | control Test(84:58)=%s | Frequency(84:32)=%s",
                 (unsigned) usages_mapped_, (unsigned) usages_ambiguous_, test_buf, freq_buf);
        if (probe_test_report_ && probe_test_report_ != 0x14) {
            ESP_LOGW(ESP32_USB_TAG, "Test usage maps to report 0x%02X, not the expected 0x14 -- this "
                                    "model differs from the fleet; verify before trusting writes.",
                     probe_test_report_);
        }
    }

    size_t expected_len = std::min(std::min(want, *data_len), sizeof(buffer));
    
    // Create USB control transfer for HID GET_REPORT
    const uint8_t bmRequestType = USB_BM_REQUEST_TYPE_DIR_IN | 
                                 USB_BM_REQUEST_TYPE_TYPE_CLASS | 
                                 USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
    const uint8_t bRequest = 0x01; // HID GET_REPORT
    const uint16_t wValue = (report_type << 8) | report_id;
    const uint16_t wIndex = device_.interface_num;
    const uint16_t wLength = expected_len;
    
    usb_transfer_t *transfer = nullptr;
    size_t transfer_size = sizeof(usb_setup_packet_t) + expected_len;
    esp_err_t ret = usb_host_transfer_alloc(transfer_size, 0, &transfer);
    if (ret != ESP_OK) {
        ESP_LOGE(ESP32_USB_TAG, "Failed to allocate transfer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Setup control transfer
    transfer->device_handle = device_.dev_hdl;
    transfer->bEndpointAddress = 0;
    transfer->num_bytes = transfer_size;
    transfer->timeout_ms = timeout_ms;
    
    // Create setup packet
    usb_setup_packet_t *setup = (usb_setup_packet_t*)transfer->data_buffer;
    setup->bmRequestType = bmRequestType;
    setup->bRequest = bRequest;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = wLength;
    
    // Use semaphore for synchronous operation
    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (!done_sem) {
        usb_host_transfer_free(transfer);
        return ESP_ERR_NO_MEM;
    }
    
    // HEAP-allocated: on timeout the transfer is still in flight, and this
    // context, the semaphore and the transfer are ABANDONED together.
    auto *ctx = new TransferCtx{done_sem, ESP_ERR_TIMEOUT, 0, -1};
    
    transfer->context = ctx;
    transfer->callback = [](usb_transfer_t *t) {
        auto *c = static_cast<TransferCtx *>(t->context);
        c->result = (t->status == USB_TRANSFER_STATUS_COMPLETED) ? ESP_OK : ESP_FAIL;
        c->actual_bytes = t->actual_num_bytes;
        xSemaphoreGive(c->sem);
    };
    
    ret = usb_host_transfer_submit_control(device_.client_hdl, transfer);
    const bool submitted_ok = (ret == ESP_OK);
    bool completed = false;
    if (submitted_ok) {
        completed = wait_for_transfer_(done_sem, timeout_ms);
        if (completed) {
            ret = ctx->result;
            if (ret == ESP_OK && ctx->actual_bytes > sizeof(usb_setup_packet_t)) {
                size_t data_received = ctx->actual_bytes - sizeof(usb_setup_packet_t);
                // Bound by expected_len as well as by the caller's buffer. The
                // transfer was allocated at sizeof(setup) + expected_len, and now
                // that wLength is derived per report those are no longer the same
                // number -- expected_len can be 2 while *data_len is 64. Bounding
                // only by *data_len would read past the END OF THE TRANSFER if a
                // device ever returns more than it was asked for.
                size_t copy_len = std::min(std::min(data_received, *data_len), expected_len);
                memcpy(data, transfer->data_buffer + sizeof(usb_setup_packet_t), copy_len);
                *data_len = copy_len;
                
                ESP_LOGD(ESP32_USB_TAG, "HID GET_REPORT success: received %zu bytes", *data_len);
            } else {
                ESP_LOGW(ESP32_USB_TAG, "HID GET_REPORT: No data received");
                *data_len = 0;
                ret = ESP_FAIL;
            }
        } else {
            ESP_LOGW(ESP32_USB_TAG, "HID GET_REPORT timeout");
            ret = ESP_ERR_TIMEOUT;
        }
    } else {
        ESP_LOGW(ESP32_USB_TAG, "Failed to submit HID GET_REPORT: %s", esp_err_to_name(ret));
    }
    
    if (submitted_ok && !completed) {
        // In flight and un-cancellable. Leak deliberately; see abandon_transfer_().
        abandon_transfer_(transfer, ctx, transfer_size, "HID GET_REPORT");
    } else {
        vSemaphoreDelete(done_sem);
        delete ctx;
        usb_host_transfer_free(transfer);
    }
    return ret;
}

esp_err_t Esp32UsbTransport::hid_set_report(uint8_t report_type, uint8_t report_id,
                                           const uint8_t* data, size_t data_len,
                                           uint32_t timeout_ms) {
    if (!device_.dev_hdl) {
        ESP_LOGE(ESP32_USB_TAG, "HID SET_REPORT: No device handle");
        return ESP_ERR_INVALID_ARG;
    }
    if (!data || data_len == 0) {
        ESP_LOGE(ESP32_USB_TAG, "HID SET_REPORT: Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    if (leak_budget_exhausted_()) return ESP_ERR_INVALID_STATE;

    ESP_LOGD(ESP32_USB_TAG, "HID SET_REPORT: type=0x%02X, id=0x%02X, len=%zu", 
             report_type, report_id, data_len);
    
    // Create USB control transfer for HID SET_REPORT
    const uint8_t bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT | 
                                 USB_BM_REQUEST_TYPE_TYPE_CLASS | 
                                 USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
    const uint8_t bRequest = 0x09; // HID SET_REPORT
    const uint16_t wValue = (report_type << 8) | report_id;
    const uint16_t wIndex = device_.interface_num;
    const uint16_t wLength = data_len;
    
    usb_transfer_t *transfer = nullptr;
    size_t transfer_size = sizeof(usb_setup_packet_t) + data_len;
    esp_err_t ret = usb_host_transfer_alloc(transfer_size, 0, &transfer);
    if (ret != ESP_OK) {
        ESP_LOGE(ESP32_USB_TAG, "Failed to allocate transfer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Setup control transfer
    transfer->device_handle = device_.dev_hdl;
    transfer->bEndpointAddress = 0;
    transfer->num_bytes = transfer_size;
    transfer->timeout_ms = timeout_ms;
    
    // Create setup packet
    usb_setup_packet_t *setup = (usb_setup_packet_t*)transfer->data_buffer;
    setup->bmRequestType = bmRequestType;
    setup->bRequest = bRequest;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = wLength;
    
    // Copy data to transfer buffer
    memcpy(transfer->data_buffer + sizeof(usb_setup_packet_t), data, data_len);
    
    // Use semaphore for synchronous operation
    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (!done_sem) {
        usb_host_transfer_free(transfer);
        return ESP_ERR_NO_MEM;
    }
    
    // HEAP-allocated: on timeout the transfer is still in flight, and this
    // context, the semaphore and the transfer are ABANDONED together.
    auto *ctx = new TransferCtx{done_sem, ESP_ERR_TIMEOUT, 0, -1};
    
    transfer->context = ctx;
    transfer->callback = [](usb_transfer_t *t) {
        auto *c = static_cast<TransferCtx *>(t->context);
        c->result = (t->status == USB_TRANSFER_STATUS_COMPLETED) ? ESP_OK : ESP_FAIL;
        xSemaphoreGive(c->sem);
    };
    
    ret = usb_host_transfer_submit_control(device_.client_hdl, transfer);
    const bool submitted_ok = (ret == ESP_OK);
    bool completed = false;
    if (submitted_ok) {
        completed = wait_for_transfer_(done_sem, timeout_ms);
        if (completed) {
            ret = ctx->result;
            if (ret == ESP_OK) {
                ESP_LOGD(ESP32_USB_TAG, "HID SET_REPORT success");
            } else {
                ESP_LOGW(ESP32_USB_TAG, "HID SET_REPORT failed");
            }
        } else {
            ESP_LOGW(ESP32_USB_TAG, "HID SET_REPORT timeout");
            ret = ESP_ERR_TIMEOUT;
        }
    } else {
        ESP_LOGW(ESP32_USB_TAG, "Failed to submit HID SET_REPORT: %s", esp_err_to_name(ret));
    }
    
    if (submitted_ok && !completed) {
        // In flight and un-cancellable. Leak deliberately; see abandon_transfer_().
        abandon_transfer_(transfer, ctx, transfer_size, "HID SET_REPORT");
    } else {
        vSemaphoreDelete(done_sem);
        delete ctx;
        usb_host_transfer_free(transfer);
    }
    return ret;
}

esp_err_t Esp32UsbTransport::get_string_descriptor(uint8_t string_index, 
                                                 std::string& result) {
    if (leak_budget_exhausted_()) return ESP_ERR_INVALID_STATE;
    result.clear();
    
    if (!device_.dev_hdl) {
        set_last_error("USB device not ready");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGD(ESP32_USB_TAG, "USB GET_STRING_DESCRIPTOR: index=%d, language_id=0x0409", string_index);
    
    // USB string descriptors can be up to 255 bytes, but typically much smaller
    const size_t max_string_len = 255;
    const uint16_t language_id = 0x0409; // English US
    
    // USB string descriptor request parameters
    const uint8_t bmRequestType = USB_BM_REQUEST_TYPE_DIR_IN | 
                                 USB_BM_REQUEST_TYPE_TYPE_STANDARD | 
                                 USB_BM_REQUEST_TYPE_RECIP_DEVICE;
    const uint8_t bRequest = USB_B_REQUEST_GET_DESCRIPTOR;
    const uint16_t wValue = (USB_B_DESCRIPTOR_TYPE_STRING << 8) | string_index;
    const uint16_t wIndex = language_id;
    const uint16_t wLength = max_string_len;
    
    usb_transfer_t *transfer = nullptr;
    size_t transfer_size = sizeof(usb_setup_packet_t) + max_string_len;
    esp_err_t ret = usb_host_transfer_alloc(transfer_size, 0, &transfer);
    if (ret != ESP_OK) {
        set_last_error("Failed to allocate string descriptor transfer: " + std::string(esp_err_to_name(ret)));
        return ret;
    }
    
    // Setup control transfer
    transfer->device_handle = device_.dev_hdl;
    transfer->bEndpointAddress = 0; // Control endpoint
    transfer->num_bytes = transfer_size;
    transfer->timeout_ms = timing::USB_CONTROL_TRANSFER_TIMEOUT_MS;
    
    // Create setup packet
    usb_setup_packet_t *setup = (usb_setup_packet_t*)transfer->data_buffer;
    setup->bmRequestType = bmRequestType;
    setup->bRequest = bRequest;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = wLength;
    
    // Use semaphore for synchronous operation
    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (!done_sem) {
        usb_host_transfer_free(transfer);
        return ESP_ERR_NO_MEM;
    }
    
    // HEAP-allocated: on timeout the transfer is still in flight, and this
    // context, the semaphore and the transfer are ABANDONED together.
    auto *ctx = new TransferCtx{done_sem, ESP_ERR_TIMEOUT, 0, -1};
    
    transfer->context = ctx;
    transfer->callback = [](usb_transfer_t *t) {
        auto *c = static_cast<TransferCtx *>(t->context);
        c->result = (t->status == USB_TRANSFER_STATUS_COMPLETED) ? ESP_OK : ESP_FAIL;
        c->actual_bytes = t->actual_num_bytes;
        xSemaphoreGive(c->sem);
    };
    
    ret = usb_host_transfer_submit_control(device_.client_hdl, transfer);
    const bool submitted_ok = (ret == ESP_OK);
    bool completed = false;
    if (submitted_ok) {
        completed = wait_for_transfer_(done_sem, timing::USB_SEMAPHORE_TIMEOUT_MS);
        if (completed) {
            ret = ctx->result;
            if (ret == ESP_OK && ctx->actual_bytes > sizeof(usb_setup_packet_t)) {
                // Parse the USB string descriptor
                uint8_t *desc_data = transfer->data_buffer + sizeof(usb_setup_packet_t);
                size_t desc_len = ctx->actual_bytes - sizeof(usb_setup_packet_t);
                
                if (desc_len >= 2) {
                    uint8_t bLength = desc_data[0];        // Total length of descriptor
                    uint8_t bDescriptorType = desc_data[1]; // Should be USB_B_DESCRIPTOR_TYPE_STRING (0x03)
                    
                    if (bDescriptorType == USB_B_DESCRIPTOR_TYPE_STRING && bLength >= 2) {
                        // USB string descriptors are UTF-16LE encoded, skip the 2-byte header
                        size_t string_data_len = std::min(static_cast<size_t>(bLength - 2), desc_len - 2);
                        uint8_t *string_data = desc_data + 2;
                        
                        // Convert UTF-16LE to ASCII (simplified, handles ASCII characters)
                        result.reserve(string_data_len / 2);
                        for (size_t i = 0; i < string_data_len; i += 2) {
                            if (i + 1 < string_data_len) {
                                uint16_t utf16_char = string_data[i] | (string_data[i + 1] << 8);
                                if (utf16_char < 128 && utf16_char > 0) { // ASCII range, non-null
                                    result += static_cast<char>(utf16_char);
                                } else if (utf16_char >= 128) {
                                    result += '?'; // Non-ASCII character placeholder
                                }
                            }
                        }
                        
                        // Trim trailing whitespace
                        while (!result.empty() && std::isspace(result.back())) {
                            result.pop_back();
                        }
                        
                        ESP_LOGI(ESP32_USB_TAG, "USB string descriptor %d: \"%s\"", string_index, result.c_str());
                    } else {
                        ESP_LOGW(ESP32_USB_TAG, "Invalid string descriptor: type=0x%02X, length=%d", bDescriptorType, bLength);
                        ret = ESP_ERR_INVALID_RESPONSE;
                    }
                } else {
                    ESP_LOGW(ESP32_USB_TAG, "String descriptor too short: %zu bytes", desc_len);
                    ret = ESP_ERR_INVALID_SIZE;
                }
            } else {
                ESP_LOGW(ESP32_USB_TAG, "USB string descriptor request failed or no data received");
                ret = ESP_FAIL;
            }
        } else {
            ESP_LOGW(ESP32_USB_TAG, "USB string descriptor request timeout");
            ret = ESP_ERR_TIMEOUT;
        }
    } else {
        ESP_LOGW(ESP32_USB_TAG, "Failed to submit string descriptor request: %s", esp_err_to_name(ret));
    }
    
    if (submitted_ok && !completed) {
        // In flight and un-cancellable. Leak deliberately; see abandon_transfer_().
        abandon_transfer_(transfer, ctx, transfer_size, "GET_DESCRIPTOR(string)");
    } else {
        vSemaphoreDelete(done_sem);
        delete ctx;
        usb_host_transfer_free(transfer);
    }
    return ret;
}

std::string Esp32UsbTransport::get_last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

// Private methods implementation

void Esp32UsbTransport::set_last_error(const std::string& error) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
    ESP_LOGW(ESP32_USB_TAG, "%s", error.c_str());
}

esp_err_t Esp32UsbTransport::setup_usb_host() {
    // Create USB library task - USB Host installation happens inside the task
    if (!usb_tasks_running_.load()) {
        usb_tasks_running_ = true;
        
        // Create USB Host Library task first
        BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib_task", 4096, this, 2, &usb_lib_task_handle_);
        if (task_created != pdTRUE) {
            ESP_LOGE(ESP32_USB_TAG, "Failed to create USB Host Library task");
            usb_tasks_running_ = false;
            return ESP_FAIL;
        }
        
        // Give USB Host Library task time to initialize
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Create USB client task
        task_created = xTaskCreate(usb_client_task, "usb_client_task", 6144, this, 3, &usb_client_task_handle_);
        if (task_created != pdTRUE) {
            ESP_LOGE(ESP32_USB_TAG, "Failed to create USB client task");
            usb_tasks_running_ = false;
            return ESP_FAIL;
        }
        
        ESP_LOGI(ESP32_USB_TAG, "USB Host tasks created successfully");
    }
    
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::teardown_usb_host() {
    // Signal tasks to stop (they will self-terminate and handle USB cleanup)
    if (usb_tasks_running_.load()) {
        ESP_LOGI(ESP32_USB_TAG, "Stopping USB Host tasks...");
        usb_tasks_running_ = false;
        
        // Wait for tasks to self-terminate
        vTaskDelay(pdMS_TO_TICKS(500)); // Give tasks time to exit cleanly
        
        usb_client_task_handle_ = nullptr;
        usb_lib_task_handle_ = nullptr;
        
        ESP_LOGI(ESP32_USB_TAG, "USB Host tasks stopped");
    }
    
    // Release interface and device
    if (device_.dev_hdl) {
        ESP_LOGI(ESP32_USB_TAG, "Cleaning up device resources");
        usb_host_interface_release(device_.client_hdl, device_.dev_hdl, device_.interface_num);
        usb_host_device_close(device_.client_hdl, device_.dev_hdl);
        device_.dev_hdl = nullptr;
    }
    
    if (device_.client_hdl) {
        ESP_LOGI(ESP32_USB_TAG, "Deregistering USB client");
        usb_host_client_deregister(device_.client_hdl);
        device_.client_hdl = nullptr;
    }
    
    // USB Host uninstallation happens inside usb_lib_task now
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::find_and_open_device() {
    // Register USB client in asynchronous mode for event-driven device detection
    usb_host_client_config_t client_config = {
        .is_synchronous = false,  // Use asynchronous mode for USB_HOST_CLIENT_EVENT_NEW_DEV events
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usb_client_event_callback,
            .callback_arg = this
        }
    };
    
    esp_err_t ret = usb_host_client_register(&client_config, &device_.client_hdl);
    if (ret != ESP_OK) {
        set_last_error("Client register failed: " + std::string(esp_err_to_name(ret)));
        return ret;
    }
    
    ESP_LOGI(ESP32_USB_TAG, "USB client registered (handle=0x%p), waiting for device connection events...", 
             device_.client_hdl);
    
    // Force immediate device enumeration check in addition to event-driven detection
    ESP_LOGI(ESP32_USB_TAG, "Performing immediate device enumeration check...");
    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay for USB stack to stabilize
    
    int num_dev = 10;
    uint8_t dev_addr_list[10];
    esp_err_t enum_ret = usb_host_device_addr_list_fill(num_dev, dev_addr_list, &num_dev);
    if (enum_ret == ESP_OK && num_dev > 0) {
        ESP_LOGI(ESP32_USB_TAG, "Found %d existing USB devices during initial enumeration:", num_dev);
        for (int i = 0; i < num_dev; i++) {
            ESP_LOGI(ESP32_USB_TAG, "  Attempting to handle existing device at address %d", dev_addr_list[i]);
            handle_new_device(dev_addr_list[i]);
        }
    } else {
        ESP_LOGI(ESP32_USB_TAG, "No existing USB devices found - waiting for connection events");
    }
    
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::get_report_descriptor(uint8_t* data, size_t* data_len,
                                                   uint32_t timeout_ms) {
    if (leak_budget_exhausted_()) return ESP_ERR_INVALID_STATE;
    if (!device_.dev_hdl) {
        set_last_error("USB device not ready");
        return ESP_ERR_INVALID_STATE;
    }
    if (!data || !data_len || *data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Prefer the length the device itself advertises. The HID class descriptor
    // (bDescriptorType 0x21) sits inside the configuration descriptor right
    // after our HID interface; its wDescriptorLength is the REPORT descriptor's
    // true size. Asking for more than that can STALL on some devices, which we
    // would then misread as "no descriptor".
    uint16_t want = (uint16_t) *data_len;
    const usb_config_desc_t *config_desc = nullptr;
    if (usb_host_get_active_config_descriptor(device_.dev_hdl, &config_desc) == ESP_OK && config_desc) {
        const uint8_t *p = (const uint8_t *) config_desc;
        uint16_t total = config_desc->wTotalLength;
        uint16_t i = 0;
        while ((uint16_t)(i + 2) <= total) {
            uint8_t bLength = p[i];
            uint8_t bDescriptorType = p[i + 1];
            if (bLength < 2) break;                       // malformed; stop walking
            if (bDescriptorType == 0x21 && (uint16_t)(i + 9) <= total && p[i + 6] == 0x22) {
                uint16_t rd_len = (uint16_t) p[i + 7] | ((uint16_t) p[i + 8] << 8);
                ESP_LOGD(ESP32_USB_TAG, "HID descriptor advertises report descriptor length %u", rd_len);
                // ⛔ FAIL SAFE -- DO NOT SOFTEN THIS INTO A TRUNCATING READ.
                // A descriptor bigger than the caller's buffer cannot be parsed
                // correctly: the item walker stops mid-item, and every report
                // still accumulating at the cut gets a PARTIAL, TOO-SMALL length.
                // An under-asked wLength is not a soft failure. The device sends
                // the whole report, the transfer buffer is short, and ESP-IDF's
                // host-controller driver ABORTS FROM ITS OWN ISR -- __assert_func
                // -> _buffer_parse, hcd_dwc.c -- with no error return and no
                // recovery. MEASURED 2026-08-23 on a CyberPower CP1500AVRLCD3
                // (0x0764:0x0601) whose descriptor is 726 bytes: it survived a
                // 512-byte read only because the single report cut at the
                // boundary happened to be one this driver never asks for.
                // Refusing here costs nothing: the caller falls back to the
                // legacy fixed length, which is exactly today's behaviour.
                if (rd_len > (uint16_t) *data_len) {
                    ESP_LOGW(ESP32_USB_TAG,
                             "Report descriptor is %u bytes but the buffer holds %u -- refusing a "
                             "truncated read; report lengths fall back to the legacy value.",
                             rd_len, (unsigned) *data_len);
                    *data_len = 0;
                    return ESP_ERR_INVALID_SIZE;
                }
                if (rd_len > 0 && rd_len < want) want = rd_len;
                break;
            }
            i = (uint16_t)(i + bLength);
        }
    } else {
        // Blind path: we could not read the config descriptor, so we do not know
        // the real length. Cap the ask at the historical 512 rather than letting
        // it follow the caller's (now larger) buffer -- growing an unbounded
        // request to a device that has already failed to describe itself is a
        // behaviour change nobody asked for, and an over-long ask is exactly
        // what some CyberPower firmware STALLs.
        if (want > 512) want = 512;
        ESP_LOGW(ESP32_USB_TAG, "No config descriptor; requesting %u bytes blind", want);
    }

    const uint8_t bmRequestType = USB_BM_REQUEST_TYPE_DIR_IN |
                                  USB_BM_REQUEST_TYPE_TYPE_STANDARD |
                                  USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
    const uint8_t bRequest = USB_B_REQUEST_GET_DESCRIPTOR;
    const uint16_t wValue = 0x2200;                       // REPORT descriptor, index 0
    const uint16_t wIndex = device_.interface_num;

    usb_transfer_t *transfer = nullptr;
    size_t transfer_size = sizeof(usb_setup_packet_t) + want;
    esp_err_t ret = usb_host_transfer_alloc(transfer_size, 0, &transfer);
    if (ret != ESP_OK) {
        set_last_error("Failed to allocate report-descriptor transfer");
        return ret;
    }

    transfer->device_handle = device_.dev_hdl;
    transfer->bEndpointAddress = 0;
    transfer->num_bytes = transfer_size;
    transfer->timeout_ms = timeout_ms;

    usb_setup_packet_t *setup = (usb_setup_packet_t*) transfer->data_buffer;
    setup->bmRequestType = bmRequestType;
    setup->bRequest = bRequest;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = want;

    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    if (!done_sem) {
        usb_host_transfer_free(transfer);
        return ESP_ERR_NO_MEM;
    }

    // HEAP-allocated: on timeout the transfer is still in flight, and this
    // context, the semaphore and the transfer are ABANDONED together.
    auto *ctx = new TransferCtx{done_sem, ESP_ERR_TIMEOUT, 0, -1};

    transfer->context = ctx;
    transfer->callback = [](usb_transfer_t *t) {
        auto *c = static_cast<TransferCtx *>(t->context);
        c->status = (int) t->status;
        c->result = (t->status == USB_TRANSFER_STATUS_COMPLETED) ? ESP_OK : ESP_FAIL;
        c->actual_bytes = t->actual_num_bytes;
        xSemaphoreGive(c->sem);
    };

    ret = usb_host_transfer_submit_control(device_.client_hdl, transfer);
    const bool submitted_ok = (ret == ESP_OK);
    bool completed = false;
    if (submitted_ok) {
        completed = wait_for_transfer_(done_sem, timeout_ms);
        if (completed) {
            // Report the RAW transfer status rather than collapsing it: a STALL
            // and a short read are otherwise indistinguishable here.
            ESP_LOGD(ESP32_USB_TAG, "GET_DESCRIPTOR(0x2200) usb_transfer_status=%d actual=%zu",
                     ctx->status, ctx->actual_bytes);
            ret = ctx->result;
            if (ret == ESP_OK && ctx->actual_bytes > sizeof(usb_setup_packet_t)) {
                size_t got = ctx->actual_bytes - sizeof(usb_setup_packet_t);
                size_t copy_len = (got < *data_len) ? got : *data_len;
                memcpy(data, transfer->data_buffer + sizeof(usb_setup_packet_t), copy_len);
                *data_len = copy_len;
            } else {
                *data_len = 0;
                if (ret == ESP_OK) ret = ESP_FAIL;        // completed, but empty
            }
        } else {
            ESP_LOGW(ESP32_USB_TAG, "GET_DESCRIPTOR(0x2200) timed out");
            *data_len = 0;
            ret = ESP_ERR_TIMEOUT;
        }
    } else {
        ESP_LOGW(ESP32_USB_TAG, "Failed to submit GET_DESCRIPTOR(0x2200): %s", esp_err_to_name(ret));
        *data_len = 0;
    }

    if (submitted_ok && !completed) {
        // In flight and un-cancellable. Leak deliberately; see abandon_transfer_().
        abandon_transfer_(transfer, ctx, transfer_size, "GET_DESCRIPTOR(0x2200)");
    } else {
        vSemaphoreDelete(done_sem);
        delete ctx;
        usb_host_transfer_free(transfer);
    }
    return ret;
}

// Have we leaked our budget? Once exhausted we stop submitting altogether: the
// device is not answering anyway, and continuing would leak without bound on a
// node that may run for months on UPS power. Reads then fail loudly, which is
// the honest report -- the UPS genuinely is not responding.
bool Esp32UsbTransport::leak_budget_exhausted_() {
    if (leaked_bytes_ < MAX_LEAKED_BYTES) return false;
    if (!leak_cap_logged_) {
        leak_cap_logged_ = true;
        ESP_LOGE(ESP32_USB_TAG,
                 "Abandoned %u bytes of un-cancellable USB transfers (budget %u). Refusing further "
                 "control transfers for this boot. The device has stopped answering; freeing an "
                 "in-flight transfer is undefined behaviour and EP0 cannot be cancelled.",
                 (unsigned) leaked_bytes_, (unsigned) MAX_LEAKED_BYTES);
    }
    return true;
}

// Wait for a control transfer to complete without starving the task watchdog.
//
// ⛔ DO NOT COLLAPSE THIS BACK INTO A SINGLE xSemaphoreTake(timeout_ms).
// ESPHome subscribes its MAIN LOOP TASK to the ESP-IDF task watchdog
// (components/esp32/hal.cpp: esp_task_wdt_add(nullptr)); the generated sdkconfig
// disables idle-task checking, so the loop task is the watched one; and the
// watchdog is CONFIG_ESP_TASK_WDT_TIMEOUT_S=5 with CONFIG_ESP_TASK_WDT_PANIC=y.
// A blocked task feeds nothing, so any single wait longer than 5 s PANICS AND
// REBOOTS THE DEVICE. set_protocol_timeout() clamps to a 5000 ms MINIMUM, i.e.
// exactly the watchdog period -- there is no configuration that avoids this.
// Feeding in slices keeps the full timeout semantics and removes the reset.
//
// ⚠ Consequence worth knowing: a wedged device now makes this node UNRESPONSIVE
// for up to protocol_timeout instead of REBOOTING it. That is the better trade
// (a reboot loses state and fixes nothing) but it is a behaviour change. The
// genuinely correct answer is to stop blocking the loop at all and make these
// reads asynchronous, which is an architecture change, not a patch.
bool Esp32UsbTransport::wait_for_transfer_(SemaphoreHandle_t sem, uint32_t timeout_ms) {
    const uint32_t SLICE_MS = 100;
    uint32_t waited = 0;
    while (waited < timeout_ms) {
        uint32_t remaining = timeout_ms - waited;
        uint32_t slice = (remaining < SLICE_MS) ? remaining : SLICE_MS;
        if (xSemaphoreTake(sem, pdMS_TO_TICKS(slice)) == pdTRUE) return true;
        waited += slice;
        App.feed_wdt();
    }
    return false;
}

// Abandon an in-flight transfer we can no longer wait for -- deliberately
// leaking the transfer, its semaphore and its context TOGETHER.
//
// ⛔ THIS IS NOT A BUG AND MUST NOT BE "TIDIED" INTO A free()/delete().
// usb_host_transfer_free() states "The transfer must not be in-flight when
// attempting to free it", and usb_transfer_t::timeout_ms is "currently not
// supported yet" -- so the stack never times a transfer out itself and a device
// that NAKs forever leaves ours genuinely queued. Cancelling is not an option:
// usb_host_endpoint_{halt,flush,clear} ALL return ESP_ERR_INVALID_ARG for
// endpoint 0 (MEASURED 2026-08-23 on a live device, with a successful submit on
// the same handle in the same function as the control). Freeing would let a
// later callback run against freed heap, write a RETURNED STACK FRAME and give
// a DELETED semaphore. Leaking is strictly the lesser evil, and it is bounded.
void Esp32UsbTransport::abandon_transfer_(usb_transfer_t *transfer, TransferCtx *ctx,
                                          size_t bytes, const char *what) {
    (void) transfer; (void) ctx;               // intentionally not freed
    leaked_bytes_ += (uint32_t) bytes;
    ESP_LOGW(ESP32_USB_TAG,
             "%s timed out and CANNOT be cancelled on EP0 -- abandoning %u bytes "
             "(%u of %u budget). This is deliberate; freeing an in-flight transfer "
             "is undefined behaviour.",
             what, (unsigned) bytes, (unsigned) leaked_bytes_, (unsigned) MAX_LEAKED_BYTES);
}

// Decide whether to (re)attempt the descriptor parse, and say so out loud.
//
// Bounded and TIME-SPACED. See the header for why per-call retries would be
// worse than the single latched attempt this replaces.
void Esp32UsbTransport::maybe_parse_report_descriptor_lengths_() {
    if (report_lengths_known_) return;

    if (report_lengths_attempts_ >= REPORT_LENGTH_MAX_ATTEMPTS) {
        // Log ONCE. By construction this lands on a later poll, so unlike the
        // parse's own output it can actually be captured over the network.
        if (!report_lengths_exhausted_logged_) {
            report_lengths_exhausted_logged_ = true;
            ESP_LOGW(ESP32_USB_TAG,
                     "Report-descriptor parse failed %u times; giving up for this connection. "
                     "Every read falls back to the legacy fixed length -- that is the pre-fix "
                     "behaviour, not a new failure, but per-report sizing is NOT in effect.",
                     (unsigned) report_lengths_attempts_);
        }
        return;
    }

    uint32_t now = millis();
    if (report_lengths_attempts_ > 0 &&
        (uint32_t)(now - report_lengths_last_attempt_ms_) < REPORT_LENGTH_RETRY_MS) {
        return;
    }

    report_lengths_last_attempt_ms_ = now;
    report_lengths_attempts_++;
    if (report_lengths_attempts_ > 1) {
        ESP_LOGI(ESP32_USB_TAG, "Retrying report-descriptor parse (attempt %u of %u)",
                 (unsigned) report_lengths_attempts_, (unsigned) REPORT_LENGTH_MAX_ATTEMPTS);
    }
    parse_report_descriptor_lengths_();
}

// Walk the HID report descriptor and record each report's declared PAYLOAD
// length, so hid_get_report() can ask for exactly that many bytes.
//
// ⚠ Walk the ITEMS. Do NOT scan for an `85 xx` byte pair to find Report IDs:
// `05 85` is Usage Page (Battery System) and collides with Report ID (0x85) on
// a naive scan. That mistake produced a wrong offset for report 0x09 on the
// CP825LCD descriptor within minutes of it being captured.
//
// Sizes ACCUMULATE: one report id can be built from several main items (0x08
// on the CP825LCD is three), so each contributes report_size * report_count
// bits to the same id.
void Esp32UsbTransport::parse_report_descriptor_lengths_() {
    report_lengths_known_ = false;
    memset(report_payload_len_, 0, sizeof(report_payload_len_));
    usage_map_known_ = false;
    memset(usage_report_id_, 0, sizeof(usage_report_id_));
    memset(usage_report_n_,  0, sizeof(usage_report_n_));
    // NB: attempt accounting and retry spacing belong to
    // maybe_parse_report_descriptor_lengths_(), which is the ONLY caller.

    // static, not stack: this runs on the USB client task, whose stack is not
    // ours to spend 512 bytes of, and it is called once per connect.
    // 1024, not 512: the largest descriptor measured on this fleet is 726 bytes
    // (CP1500AVRLCD3), and a second unit sits at 510 -- two bytes under the old
    // ceiling. Anything that still does not fit is REFUSED by
    // get_report_descriptor() rather than truncated; see the fail-safe there.
    static uint8_t desc[1024];
    size_t len = sizeof(desc);
    esp_err_t ret = get_report_descriptor(desc, &len, 2000);
    if (ret == ESP_ERR_TIMEOUT) {
        // A timeout means we ABANDONED a transfer (it cannot be cancelled). Retrying
        // would leak again, and a device that does not answer a descriptor request in
        // 2 s is not going to answer the next one either. Burn the remaining attempts.
        report_lengths_attempts_ = REPORT_LENGTH_MAX_ATTEMPTS;
    }
    if (ret != ESP_OK || len == 0) {
        ESP_LOGW(ESP32_USB_TAG,
                 "Report descriptor unavailable (%s, len=%u) -- hid_get_report falls back to "
                 "the legacy fixed length. This is the status quo, not a new failure.",
                 esp_err_to_name(ret), (unsigned) len);
        return;
    }

    // bits[type][id], accumulated; converted to bytes at the end.
    static uint32_t bits[4][256];
    memset(bits, 0, sizeof(bits));

    uint16_t rid = 0, rsize = 0, rcount = 0;
    // Usage Page is GLOBAL (persists until changed); Usage is LOCAL (consumed by
    // the next Main item). Conflating the two scopes is the classic way to
    // mis-walk a descriptor, so they are tracked separately and pending_usages
    // is cleared at every Main item whether or not it was one we cared about.
    uint16_t upage = 0;
    uint8_t  pending_row[16];      // which page row (0 = 0x84, 1 = 0x85)
    uint8_t  pending_usage[16];    // the usage id itself
    uint8_t  n_pending = 0;
    size_t i = 0;
    while (i < len) {
        uint8_t b = desc[i++];
        if (b == 0xFE) {                      // long item: skip it wholesale
            if (i >= len) break;
            uint8_t datasize = desc[i];
            i += 2 + datasize;
            continue;
        }
        uint8_t sz = b & 0x03;
        if (sz == 3) sz = 4;                  // 0b11 encodes 4 bytes, not 3
        uint8_t type = (b >> 2) & 0x03;       // 0=Main 1=Global 2=Local
        uint8_t tag  = b & 0xFC;
        if (i + sz > len) break;              // truncated descriptor
        uint32_t val = 0;
        for (uint8_t k = 0; k < sz; k++) val |= ((uint32_t) desc[i + k]) << (8 * k);
        i += sz;

        if (type == 1) {                      // Global
            if      (tag == 0x84) rid    = (uint16_t) val;   // Report ID
            else if (tag == 0x74) rsize  = (uint16_t) val;   // Report Size (bits)
            else if (tag == 0x94) rcount = (uint16_t) val;   // Report Count
            else if (tag == 0x04) upage  = (uint16_t) val;   // Usage Page
        } else if (type == 2) {               // Local
            // 0x08 Usage. A 4-byte Usage is an EXTENDED usage carrying its own
            // page in the high 16 bits, which overrides the global page for that
            // one item -- ignoring that would file it under the wrong page.
            if (tag == 0x08 && n_pending < sizeof(pending_usage)) {
                uint16_t item_page = (sz == 4) ? (uint16_t) (val >> 16) : upage;
                uint8_t  item_usg  = (uint8_t) (val & 0xFF);
                int8_t   row = -1;
                if      (item_page == USAGE_PAGE_POWER_DEVICE)   row = 0;
                else if (item_page == USAGE_PAGE_BATTERY_SYSTEM) row = 1;
                if (row >= 0) {
                    pending_row[n_pending]   = (uint8_t) row;
                    pending_usage[n_pending] = item_usg;
                    n_pending++;
                }
            }
            // Usage Minimum/Maximum (0x18/0x28) describe RANGES -- deliberately
            // not mapped. Nothing this driver reads is declared as a range, and
            // expanding one would put dozens of synthetic entries in the map.
        } else if (type == 0) {               // Main
            uint8_t rt = 0;
            if      (tag == 0x80) rt = HID_REPORT_TYPE_INPUT;
            else if (tag == 0x90) rt = HID_REPORT_TYPE_OUTPUT;
            else if (tag == 0xB0) rt = HID_REPORT_TYPE_FEATURE;
            if (rt != 0 && rid < 256) {
                bits[rt][rid] += (uint32_t) rsize * (uint32_t) rcount;
                for (uint8_t u = 0; u < n_pending; u++) {
                    uint8_t row = pending_row[u], usg = pending_usage[u];
                    if (usage_report_id_[row][usg] == 0) {
                        usage_report_id_[row][usg] = (uint8_t) rid;
                        usage_report_n_[row][usg]  = 1;
                    } else if (usage_report_id_[row][usg] != (uint8_t) rid) {
                        // A SECOND report declares it -> ambiguous from here on.
                        // Saturate rather than wrap; the only thing callers ask is
                        // "is this exactly one?".
                        if (usage_report_n_[row][usg] < 255) usage_report_n_[row][usg]++;
                    }
                    // Same usage in the SAME report as both Feature and Input is
                    // normal on this fleet (0x0B, 0x10, 0x14 all do it) and is NOT
                    // ambiguity -- hence the != rid guard above.
                }
            }
            // Local items are consumed by EVERY Main item, including Collection
            // and End Collection, whether or not we mapped anything from them.
            // Leaving them pending would leak a Collection's usage onto the next
            // Input/Feature and file a report under a usage it never declared.
            n_pending = 0;
        }
    }

    uint16_t declared = 0;
    for (uint8_t t = 1; t < 4; t++) {
        for (uint16_t id = 0; id < 256; id++) {
            if (bits[t][id] == 0) continue;
            uint32_t nbytes = (bits[t][id] + 7) / 8;
            if (nbytes > 255) nbytes = 255;   // cannot exceed the array's type
            report_payload_len_[t][id] = (uint8_t) nbytes;
            declared++;
        }
    }

    uint16_t usages_mapped = 0, usages_ambiguous = 0;
    for (uint8_t row = 0; row < 2; row++) {
        for (uint16_t u = 0; u < 256; u++) {
            if (usage_report_id_[row][u] == 0) continue;
            usages_mapped++;
            if (usage_report_n_[row][u] > 1) usages_ambiguous++;
        }
    }
    usage_map_known_ = (usages_mapped > 0);

    report_lengths_known_ = (declared > 0);
    if (report_lengths_known_) report_lengths_known_at_ms_ = millis();
    ESP_LOGI(ESP32_USB_TAG,
             "Report descriptor parsed: %u bytes, %u report(s) declared -- "
             "hid_get_report will use each report's own length",
             (unsigned) len, (unsigned) declared);
    if (!report_lengths_known_) {
        ESP_LOGW(ESP32_USB_TAG, "Descriptor parsed but declared NOTHING -- falling back to the "
                                "legacy fixed length for every report.");
    }

    // ★ SELF-PROVING SUMMARY -- STASHED, NOT LOGGED HERE. See the header: this
    // function runs inside the first poll and nothing it prints can be captured.
    // The reading it supports is an ABSENCE ("this device declares no frequency"),
    // which is indistinguishable from a broken map -- so it carries a KNOWN
    // POSITIVE alongside: Power Device 0x58 (Test) is declared by every CyberPower
    // unit here as report 0x14, the one the self-test button writes to. If Test
    // resolves and Frequency does not, the map works. If Test does not resolve,
    // distrust the whole line.
    usages_mapped_    = usages_mapped;
    usages_ambiguous_ = usages_ambiguous;
    probe_test_report_ = 0;
    probe_freq_report_ = 0;
    find_report_for_usage(0x84, 0x58, &probe_test_report_);
    find_report_for_usage(0x84, 0x32, &probe_freq_report_);
}

bool Esp32UsbTransport::find_report_for_usage(uint16_t usage_page, uint8_t usage,
                                              uint8_t* report_id) const {
    if (!usage_map_known_) return false;
    int8_t row = -1;
    if      (usage_page == USAGE_PAGE_POWER_DEVICE)   row = 0;
    else if (usage_page == USAGE_PAGE_BATTERY_SYSTEM) row = 1;
    if (row < 0) return false;
    if (usage_report_id_[row][usage] == 0) return false;
    if (usage_report_n_[row][usage] != 1) return false;   // ambiguous -- see the header
    if (report_id) *report_id = usage_report_id_[row][usage];
    return true;
}

esp_err_t Esp32UsbTransport::claim_interface() {
    const usb_config_desc_t *config_desc;
    esp_err_t ret = usb_host_get_active_config_descriptor(device_.dev_hdl, &config_desc);
    if (ret != ESP_OK) {
        set_last_error("Failed to get config descriptor");
        return ret;
    }
    
    // Find HID interface
    const usb_intf_desc_t *intf_desc = nullptr;
    int offset = 0;
    
    for (int i = 0; i < config_desc->bNumInterfaces; i++) {
        intf_desc = usb_parse_interface_descriptor(config_desc, i, 0, &offset);
        if (intf_desc && intf_desc->bInterfaceClass == USB_CLASS_HID) {
            device_.interface_num = intf_desc->bInterfaceNumber;
            break;
        }
    }
    
    if (!intf_desc || intf_desc->bInterfaceClass != USB_CLASS_HID) {
        set_last_error("No HID interface found");
        return ESP_ERR_NOT_FOUND;
    }
    
    ret = usb_host_interface_claim(device_.client_hdl, device_.dev_hdl, 
                                  device_.interface_num, 0);
    if (ret != ESP_OK) {
        set_last_error("Failed to claim interface: " + std::string(esp_err_to_name(ret)));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::find_endpoints() {
    const usb_config_desc_t *config_desc;
    esp_err_t ret = usb_host_get_active_config_descriptor(device_.dev_hdl, &config_desc);
    if (ret != ESP_OK) {
        return ret;
    }
    
    int offset = 0;
    const usb_intf_desc_t *intf_desc = usb_parse_interface_descriptor(
        config_desc, device_.interface_num, 0, &offset);
    
    if (!intf_desc) {
        set_last_error("Interface descriptor not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Parse endpoints correctly using ESP-IDF approach
    const usb_ep_desc_t *ep_desc = nullptr;
    int ep_offset = offset;
    
    ESP_LOGD(ESP32_USB_TAG, "Interface has %d endpoints", intf_desc->bNumEndpoints);
    
    for (int i = 0; i < intf_desc->bNumEndpoints; i++) {
        ep_desc = usb_parse_endpoint_descriptor_by_index(intf_desc, i, config_desc->wTotalLength, &ep_offset);
        if (ep_desc) {
            if (USB_EP_DESC_GET_EP_DIR(ep_desc)) {
                // IN endpoint (device to host)
                device_.ep_in = ep_desc->bEndpointAddress;
                device_.max_packet_size_in = ep_desc->wMaxPacketSize;
                ESP_LOGD(ESP32_USB_TAG, "Found IN endpoint: 0x%02X (max packet size: %d)",
                         device_.ep_in, device_.max_packet_size_in);
            } else {
                // OUT endpoint (host to device)
                device_.ep_out = ep_desc->bEndpointAddress;
                device_.max_packet_size_out = ep_desc->wMaxPacketSize;
                ESP_LOGD(ESP32_USB_TAG, "Found OUT endpoint: 0x%02X (max packet size: %d)",
                         device_.ep_out, device_.max_packet_size_out);
            }
        } else {
            ESP_LOGW(ESP32_USB_TAG, "Failed to parse endpoint %d", i);
        }
    }

    if (device_.ep_in == 0) {
        set_last_error("No IN endpoint found");
        return ESP_ERR_NOT_FOUND;
    }

    // Detect input-only devices (no OUT endpoint)
    if (device_.ep_out == 0) {
        ESP_LOGW(ESP32_USB_TAG, "INPUT-ONLY HID device detected - no OUT endpoint available");
        ESP_LOGI(ESP32_USB_TAG, "Device supports HID GET_REPORT only (no SET_REPORT)");
    } else {
        ESP_LOGD(ESP32_USB_TAG, "Bidirectional device detected - has both IN and OUT endpoints");
    }
    
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::submit_control_transfer(uint8_t bmRequestType, uint8_t bRequest,
                                                   uint16_t wValue, uint16_t wIndex,
                                                   uint8_t* data, size_t data_len,
                                                   uint32_t timeout_ms) {
    std::lock_guard<std::mutex> lock(device_mutex_);
    
    if (!device_.dev_hdl) {
        return ESP_ERR_INVALID_STATE;
    }
    
    usb_transfer_t *transfer;
    esp_err_t ret = usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + data_len, 0, &transfer);
    if (ret != ESP_OK) {
        set_last_error("Transfer alloc failed: " + std::string(esp_err_to_name(ret)));
        return ret;
    }
    
    // Setup packet
    usb_setup_packet_t *setup = (usb_setup_packet_t *)transfer->data_buffer;
    setup->bmRequestType = bmRequestType;
    setup->bRequest = bRequest;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = data_len;
    
    if (data_len > 0 && data) {
        if (bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) {
            // IN transfer - device to host
            memset(transfer->data_buffer + sizeof(usb_setup_packet_t), 0, data_len);
        } else {
            // OUT transfer - host to device
            memcpy(transfer->data_buffer + sizeof(usb_setup_packet_t), data, data_len);
        }
    }
    
    transfer->device_handle = device_.dev_hdl;
    transfer->bEndpointAddress = 0; // Control endpoint
    transfer->callback = nullptr;
    transfer->context = nullptr;
    transfer->num_bytes = sizeof(usb_setup_packet_t) + data_len;
    transfer->timeout_ms = timeout_ms;
    
    ret = usb_host_transfer_submit_control(device_.client_hdl, transfer);
    if (ret != ESP_OK) {
        usb_host_transfer_free(transfer);
        set_last_error("Control transfer submit failed: " + std::string(esp_err_to_name(ret)));
        return ret;
    }
    
    // Copy response data back
    if (data_len > 0 && data && (bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN)) {
        memcpy(data, transfer->data_buffer + sizeof(usb_setup_packet_t), data_len);
    }
    
    usb_host_transfer_free(transfer);
    return ESP_OK;
}

void Esp32UsbTransport::usb_client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg) {
    Esp32UsbTransport* transport = static_cast<Esp32UsbTransport*>(arg);
    
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(ESP32_USB_TAG, "New USB device detected: address=%d", event_msg->new_dev.address);
            transport->handle_new_device(event_msg->new_dev.address);
            break;
            
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(ESP32_USB_TAG, "USB device disconnected");
            transport->handle_device_gone(event_msg->dev_gone.dev_hdl);
            break;
            
        default:
            ESP_LOGW(ESP32_USB_TAG, "Unhandled USB client event: %d", event_msg->event);
            break;
    }
}

void Esp32UsbTransport::handle_new_device(uint8_t dev_addr) {
    ESP_LOGI(ESP32_USB_TAG, "Handling new USB device at address %d", dev_addr);
    
    std::lock_guard<std::mutex> lock(device_mutex_);
    
    // Check if we already have a device connected
    if (device_.dev_hdl != nullptr) {
        ESP_LOGW(ESP32_USB_TAG, "Device already connected - skipping new device at address %d", dev_addr);
        return;
    }
    
    // Open the device
    esp_err_t ret = usb_host_device_open(device_.client_hdl, dev_addr, &device_.dev_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(ESP32_USB_TAG, "Failed to open device at address %d: %s", dev_addr, esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(ESP32_USB_TAG, "Successfully opened USB device at address %d", dev_addr);
    
    // Get device information
    usb_device_info_t dev_info;
    ret = usb_host_device_info(device_.dev_hdl, &dev_info);
    if (ret != ESP_OK) {
        ESP_LOGE(ESP32_USB_TAG, "Failed to get device info: %s", esp_err_to_name(ret));
        usb_host_device_close(device_.client_hdl, device_.dev_hdl);
        device_.dev_hdl = nullptr;
        return;
    }
    
    device_.address = dev_addr;
    device_.speed = dev_info.speed;
    
    // Get device descriptor to extract VID/PID
    const usb_device_desc_t* device_desc;
    ret = usb_host_get_device_descriptor(device_.dev_hdl, &device_desc);
    if (ret == ESP_OK) {
        device_.vendor_id = device_desc->idVendor;
        device_.product_id = device_desc->idProduct;
        
        ESP_LOGI(ESP32_USB_TAG, "USB device opened: VID=0x%04X, PID=0x%04X, Speed=%d", 
                 device_.vendor_id, device_.product_id, dev_info.speed);
        
        // Check if this is a UPS device (HID class)
        if (device_desc->bDeviceClass == USB_CLASS_HID || 
            device_desc->bDeviceClass == 0x00) { // Device class defined at interface level
            
            // Try to claim HID interface and find endpoints
            ret = claim_interface();
            if (ret == ESP_OK) {
                ret = find_endpoints();
                if (ret == ESP_OK) {
                    connected_ = true;
                    // ⛔ DO NOT parse the descriptor here. We are running INSIDE
                    // usb_host_client_handle_events() -- handle_new_device() is
                    // dispatched from it -- and get_report_descriptor() blocks on
                    // a semaphore that only that same event loop can give. Calling
                    // it here is a self-deadlock: the transfer cannot complete, it
                    // times out, and every read silently falls back to the legacy
                    // length. MEASURED 2026-08-22; the symptom is indistinguishable
                    // from the fix simply not working, because the diagnostic it
                    // logs is emitted before the API log subscription exists.
                    // Arm it instead; hid_get_report() parses lazily on first use,
                    // from the component's own task, where the read is proven.
                    report_lengths_known_ = false;
                    report_lengths_attempts_ = 0;
                    report_lengths_last_attempt_ms_ = 0;
                    report_lengths_known_at_ms_ = 0;
                    report_lengths_exhausted_logged_ = false;
                    report_lengths_confirmed_logged_ = false;
                    ESP_LOGI(ESP32_USB_TAG, "UPS device successfully configured and ready");
                    return;
                }
            }
        } else {
            ESP_LOGW(ESP32_USB_TAG, "Connected device is not a HID device (class=0x%02X)", device_desc->bDeviceClass);
        }
    }
    
    // Clean up on failure
    usb_host_device_close(device_.client_hdl, device_.dev_hdl);
    device_.dev_hdl = nullptr;
}

void Esp32UsbTransport::handle_device_gone(usb_device_handle_t dev_hdl) {
    ESP_LOGI(ESP32_USB_TAG, "Handling USB device disconnection");
    
    std::lock_guard<std::mutex> lock(device_mutex_);
    
    if (device_.dev_hdl == dev_hdl) {
        connected_ = false;
        
        // Clean up device resources
        if (device_.dev_hdl) {
            usb_host_interface_release(device_.client_hdl, device_.dev_hdl, device_.interface_num);
            usb_host_device_close(device_.client_hdl, device_.dev_hdl);
            device_.dev_hdl = nullptr;
        }
        
        // Reset device info
        device_.address = 0;
        device_.vendor_id = 0;
        device_.product_id = 0;
        
        ESP_LOGI(ESP32_USB_TAG, "USB device disconnected and cleaned up");
    }
}

void Esp32UsbTransport::usb_lib_task(void* arg) {
    Esp32UsbTransport* transport = static_cast<Esp32UsbTransport*>(arg);
    
    ESP_LOGI(ESP32_USB_TAG, "USB Host Library task starting...");
    
    // Initialize USB Host library inside task (following working prototype)
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK) {
        ESP_LOGE(ESP32_USB_TAG, "USB Host install failed: %s", esp_err_to_name(ret));
        transport->usb_tasks_running_ = false; // Stop other tasks
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(ESP32_USB_TAG, "USB Host library installed successfully");
    
    // Main USB Host event loop - following working prototype pattern
    bool has_clients = true;
    bool has_devices = false;
    
    while (has_clients && transport->usb_tasks_running_.load()) {
        uint32_t event_flags;
        ret = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        
        if (ret != ESP_OK) {
            ESP_LOGE(ESP32_USB_TAG, "USB Host event handling failed: %s", esp_err_to_name(ret));
            continue;
        }

        // Handle USB Host events (following ESP-IDF v5.4 patterns)
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(ESP32_USB_TAG, "No more USB clients");
            if (usb_host_device_free_all() == ESP_OK) {
                ESP_LOGI(ESP32_USB_TAG, "All devices freed");
                has_clients = false;
            } else {
                ESP_LOGI(ESP32_USB_TAG, "Waiting for devices to be freed");
                has_devices = true;
            }
        }
        
        if (has_devices && (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)) {
            ESP_LOGI(ESP32_USB_TAG, "All devices freed");
            has_clients = false;
        }
    }
    
    // Cleanup USB Host library
    ESP_LOGI(ESP32_USB_TAG, "Uninstalling USB Host library");
    usb_host_uninstall();
    
    ESP_LOGI(ESP32_USB_TAG, "USB Host Library task ending");
    vTaskDelete(nullptr);
}

void Esp32UsbTransport::usb_client_task(void* arg) {
    Esp32UsbTransport* transport = static_cast<Esp32UsbTransport*>(arg);
    
    ESP_LOGI(ESP32_USB_TAG, "USB client task started");
    
    while (transport->usb_tasks_running_.load()) {
        if (transport->device_.client_hdl) {
            // Use shorter timeout for more responsive event processing
            esp_err_t ret = usb_host_client_handle_events(transport->device_.client_hdl, timing::USB_CLIENT_EVENT_TIMEOUT_MS);
            
            if (ret == ESP_ERR_TIMEOUT) {
                // Timeout is normal - continue processing
            } else if (ret != ESP_OK) {
                ESP_LOGW(ESP32_USB_TAG, "USB client event handling failed: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(100)); // Brief delay on error
            }
        } else {
            ESP_LOGD(ESP32_USB_TAG, "No USB client handle available");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // Small delay to prevent busy-waiting
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(ESP32_USB_TAG, "USB client task stopping");
    vTaskDelete(nullptr);
}

} // namespace ups_hid
} // namespace esphome

#endif // USE_ESP32