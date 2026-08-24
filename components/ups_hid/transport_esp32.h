#pragma once

#include "transport_interface.h"

#ifdef USE_ESP32
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <mutex>
#include <set>
#include <atomic>

namespace esphome {
namespace ups_hid {

/**
 * ESP32 USB Transport Implementation
 * 
 * Concrete implementation using ESP-IDF USB Host API
 * Handles all ESP32-specific USB communication details
 */
class Esp32UsbTransport : public IUsbTransport {
public:
    Esp32UsbTransport();
    ~Esp32UsbTransport() override;
    
    // IUsbTransport implementation
    esp_err_t initialize() override;
    esp_err_t deinitialize() override;
    
    bool is_connected() const override;
    uint16_t get_vendor_id() const override;
    uint16_t get_product_id() const override;
    
    esp_err_t hid_get_report(uint8_t report_type, uint8_t report_id, 
                           uint8_t* data, size_t* data_len, 
                           uint32_t timeout_ms = 1000) override;
    
    esp_err_t hid_set_report(uint8_t report_type, uint8_t report_id,
                           const uint8_t* data, size_t data_len,
                           uint32_t timeout_ms = 1000) override;
    
    esp_err_t get_string_descriptor(uint8_t string_index, 
                                  std::string& result) override;

    esp_err_t get_report_descriptor(uint8_t* data, size_t* data_len,
                                    uint32_t timeout_ms = 1000) override;

    bool find_report_for_usage(uint16_t usage_page, uint8_t usage,
                               uint8_t* report_id) const override;
    bool usage_map_known() const override { return usage_map_known_; }
    
    std::string get_last_error() const override;

private:
    // USB device structure
    struct UsbDevice {
        usb_host_client_handle_t client_hdl{nullptr};
        usb_device_handle_t dev_hdl{nullptr};
        uint8_t address{0};
        uint8_t interface_num{0};
        uint8_t ep_in{0};
        uint8_t ep_out{0};
        uint16_t vendor_id{0};
        uint16_t product_id{0};
        uint16_t max_packet_size_in{0};
        uint16_t max_packet_size_out{0};
        usb_speed_t speed{USB_SPEED_LOW};
    };
    
    UsbDevice device_;
    mutable std::mutex device_mutex_;

    // ── Declared report lengths, parsed from the device's own HID report
    // descriptor at connect. Indexed [report_type][report_id]; the value is the
    // PAYLOAD in bytes, so a request is payload + 1 for the leading report-id
    // byte (NUT's rule, drivers/libhid.c:131). 0 means "not declared", which is
    // the fail-safe path back to the legacy fixed length. Types are 1/2/3, so
    // index 0 is unused and the array is deliberately [4].
    uint8_t  report_payload_len_[4][256]{};
    bool     report_lengths_known_{false};

    // Reverse map built by the same descriptor walk: (page, usage) -> report id.
    // Row 0 is Power Device (0x84), row 1 is Battery System (0x85); those are the
    // only pages anything reads from, and the vendor page 0xFF01 is reached by a
    // report ID we already know. 512 B of .bss, populated once per connect.
    //
    // usage_report_n_ is NOT redundant with a zero check: it counts how many
    // DISTINCT report IDs declare the usage, so the lookup can refuse an
    // ambiguous one. Report 0 is not a legal HID report id here, so 0 doubles as
    // "absent" in usage_report_id_.
    static constexpr uint8_t USAGE_PAGE_POWER_DEVICE   = 0x84;
    static constexpr uint8_t USAGE_PAGE_BATTERY_SYSTEM = 0x85;
    uint8_t  usage_report_id_[2][256]{};
    uint8_t  usage_report_n_[2][256]{};
    bool     usage_map_known_{false};
    // ⚠ The usage-map summary CANNOT be logged where it is computed. The
    // descriptor parse runs inside the first poll, before the API/log connection
    // exists, so anything it prints is unreachable over the network -- measured
    // 2026-08-23 on device D: 248 log lines captured, ZERO containing it. That is
    // the same trap report_lengths_confirmed_logged_ already exists to dodge, and
    // it was walked into anyway by the person who read that comment. These carry
    // the result forward to the DELAYED one-shot in hid_get_report().
    uint16_t usages_mapped_{0};
    uint16_t usages_ambiguous_{0};
    uint8_t  probe_test_report_{0};      // 0x84:0x58 Test -- the known positive
    uint8_t  probe_freq_report_{0};      // 0x84:0x32 Frequency -- the real query
    bool     usage_map_logged_{false};

    // ── Parse scheduling. The parse used to be a single latched attempt, which
    // made ONE transient failure permanent for the whole boot -- and silent,
    // because the attempt happens inside the first update(), whose log output
    // cannot be captured over the network (the API subscription is not live
    // yet). A device could therefore run with the fix doing nothing at all and
    // look identical to one where it worked. MEASURED 2026-08-23 on a
    // CP1500AVRLCD3: every production read still asked the legacy 64.
    // Retries are TIME-SPACED, not per-call: hid_get_report() is called ~29
    // times per poll and a failed parse blocks for up to 2 s, so retrying on
    // each call would burn every attempt inside the first poll and block the
    // main loop for ~6 s. Spacing them puts attempts 2+ on later polls, which
    // is also what makes their outcome capturable at all.
    static constexpr uint8_t  REPORT_LENGTH_MAX_ATTEMPTS = 3;
    static constexpr uint32_t REPORT_LENGTH_RETRY_MS = 5000;
    // Separate constant on purpose: this one only delays a LOG line past the
    // first (uncapturable) poll. Sharing the retry interval would mean tuning
    // the retry silently changed what you can observe.
    static constexpr uint32_t REPORT_LENGTH_CONFIRM_DELAY_MS = 5000;
    uint8_t  report_lengths_attempts_{0};
    uint32_t report_lengths_last_attempt_ms_{0};
    uint32_t report_lengths_known_at_ms_{0};
    bool     report_lengths_exhausted_logged_{false};
    bool     report_lengths_confirmed_logged_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> initialized_{false};
    
    // USB Host Library management
    TaskHandle_t usb_lib_task_handle_{nullptr};
    TaskHandle_t usb_client_task_handle_{nullptr};
    std::atomic<bool> usb_tasks_running_{false};
    
    // Error handling
    mutable std::mutex error_mutex_;
    std::string last_error_;
    
    // Private methods
    static void usb_lib_task(void* arg);
    static void usb_client_task(void* arg);
    static void usb_client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);
    
    void handle_new_device(uint8_t dev_addr);
    void handle_device_gone(usb_device_handle_t dev_hdl);
    
    esp_err_t setup_usb_host();
    esp_err_t teardown_usb_host();
    esp_err_t find_and_open_device();
    esp_err_t claim_interface();
    void parse_report_descriptor_lengths_();
    void maybe_parse_report_descriptor_lengths_();

    // ── Control-transfer completion context.
    // HEAP-allocated, not a stack local, because a transfer we time out on is
    // still IN FLIGHT: ESP-IDF cannot cancel it (usb_host_endpoint_halt/flush/
    // clear all return ESP_ERR_INVALID_ARG for EP0 -- MEASURED 2026-08-23), and
    // usb_host_transfer_free() states "The transfer must not be in-flight".
    // So the transfer, its semaphore and this context are ABANDONED together
    // and their lifetimes must not end with the calling frame.
    struct TransferCtx {
        SemaphoreHandle_t sem;
        esp_err_t result;
        size_t actual_bytes;
        int status;
    };

    // Wait for completion WITHOUT starving the task watchdog. ESPHome subscribes
    // its main loop task to the WDT (esp32/hal.cpp: esp_task_wdt_add(nullptr)),
    // idle-task checking is off, and the WDT is 5 s with PANIC=y -- so a single
    // xSemaphoreTake() longer than 5 s RESETS THE DEVICE. protocol_timeout's
    // minimum clamp is exactly 5000 ms, so no configuration avoids it.
    bool wait_for_transfer_(SemaphoreHandle_t sem, uint32_t timeout_ms);
    bool leak_budget_exhausted_();

    // Deliberately leak an un-cancellable in-flight transfer. See the .cpp.
    void abandon_transfer_(usb_transfer_t *transfer, TransferCtx *ctx, size_t bytes,
                           const char *what);

    // Bounded in BYTES rather than in count, because the two paths leak very
    // different amounts (a descriptor transfer is ~734 B, a report one ~72 B),
    // and bytes are the quantity actually at risk. NOT reset on reconnect: a
    // per-connection bound is re-armed by every re-enumeration, so a flapping
    // device would leak without limit -- and these run for months on UPS power.
    static constexpr uint32_t MAX_LEAKED_BYTES = 8192;
    uint32_t leaked_bytes_{0};
    bool     leak_cap_logged_{false};
    esp_err_t find_endpoints();
    
    void set_last_error(const std::string& error);
    esp_err_t submit_control_transfer(uint8_t bmRequestType, uint8_t bRequest,
                                    uint16_t wValue, uint16_t wIndex,
                                    uint8_t* data, size_t data_len,
                                    uint32_t timeout_ms);
};

} // namespace ups_hid
} // namespace esphome

#endif // USE_ESP32