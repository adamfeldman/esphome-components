#pragma once

#include "esp_err.h"
#include <vector>
#include <cstdint>
#include <memory>

namespace esphome {
namespace ups_hid {

/**
 * Abstract USB Transport Interface
 * 
 * Provides a clean abstraction over USB communication, allowing for
 * different implementations (ESP32 hardware, simulation, etc.)
 * 
 * Design Pattern: Strategy Pattern for transport selection
 */
class IUsbTransport {
public:
    virtual ~IUsbTransport() = default;
    
    // Transport lifecycle
    virtual esp_err_t initialize() = 0;
    virtual esp_err_t deinitialize() = 0;
    
    // Device management
    virtual bool is_connected() const = 0;
    virtual uint16_t get_vendor_id() const = 0;
    virtual uint16_t get_product_id() const = 0;
    
    // HID communication
    virtual esp_err_t hid_get_report(uint8_t report_type, uint8_t report_id, 
                                   uint8_t* data, size_t* data_len, 
                                   uint32_t timeout_ms = 1000) = 0;
    
    virtual esp_err_t hid_set_report(uint8_t report_type, uint8_t report_id,
                                   const uint8_t* data, size_t data_len,
                                   uint32_t timeout_ms = 1000) = 0;
    
    // String descriptors
    virtual esp_err_t get_string_descriptor(uint8_t string_index, 
                                          std::string& result) = 0;

    // HID REPORT descriptor -- standard GET_DESCRIPTOR, wValue 0x2200.
    // Deliberately NOT pure virtual: a transport that cannot serve it (the
    // simulation transport) then needs no change and cannot fail to build.
    virtual esp_err_t get_report_descriptor(uint8_t* data, size_t* data_len,
                                            uint32_t timeout_ms = 1000) {
        (void) data; (void) timeout_ms;
        if (data_len) *data_len = 0;
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Which report declares a given (usage page, usage)?  Answers the question
    // "where do I read X from" from the descriptor the DEVICE published, instead
    // of guessing report IDs.
    //
    // ⛔ RETURNS FALSE WHEN THE USAGE IS DECLARED BY MORE THAN ONE REPORT, and
    // that is the whole point of the API rather than a limitation of it. HID
    // disambiguates repeated usages by COLLECTION -- UPS.Battery.Voltage vs
    // UPS.Input.Voltage vs UPS.Output.Voltage are all `0x84:0x30`, and all three
    // exist on this fleet (reports 0x0A, 0x0F, 0x12). A lookup that silently
    // returned the first match would rebuild the exact defect this replaces:
    // confidently reading the wrong report. Ambiguous means "ask by report ID,
    // you know which collection you want"; only unique usages are answerable here.
    //
    // Deliberately NOT pure virtual, matching get_report_descriptor() above: a
    // transport that cannot serve it needs no change and cannot fail to build.
    // The default is "I do not know", never "not declared" -- callers must not
    // read a false return as evidence about the device.
    virtual bool find_report_for_usage(uint16_t usage_page, uint8_t usage,
                                       uint8_t* report_id) const {
        (void) usage_page; (void) usage; (void) report_id;
        return false;
    }

    // Did the usage map get built at all?  Separates "the device does not declare
    // this" from "we never parsed a descriptor", which find_report_for_usage()
    // cannot distinguish on its own.
    virtual bool usage_map_known() const { return false; }
    
    // Error information
    virtual std::string get_last_error() const = 0;
};

// Forward declaration - implementation in usb_transport_factory.h
class UsbTransportFactory;

} // namespace ups_hid
} // namespace esphome