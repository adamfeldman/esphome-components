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
    // DIAGNOSTIC ONLY -- never merge into the fix branch. See the .cpp.
    void probe_ep0_cancel_();
    bool ep0_probe_done_{false};
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