#include "ups_hid.h"
#include "constants_ups.h"
#include "transport_factory.h"
#include "transport_simulation.h"
#ifdef USE_ESP32
#include "transport_esp32.h"
#endif
#include "protocol_factory.h"
#include "protocol_apc.h"
#include "protocol_cyberpower.h"
 
#include "protocol_generic.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <functional>
#include <cmath>

namespace esphome {
namespace ups_hid {

void UpsHidComponent::setup() {
  ESP_LOGCONFIG(TAG, log_messages::SETTING_UP);
  
  if (!initialize_transport()) {
    ESP_LOGE(TAG, log_messages::TRANSPORT_INIT_FAILED);
    mark_failed();
    return;
  }
  
  // Protocol detection is deferred to update() method to handle asynchronous USB enumeration
  ESP_LOGCONFIG(TAG, log_messages::SETUP_COMPLETE);
}

void UpsHidComponent::update() {
  if (!transport_ || !transport_->is_connected()) {
    // Device not connected yet - normal during startup or after disconnection
    ESP_LOGD(TAG, log_messages::WAITING_FOR_DEVICE);
    return;
  }
  
  // ── cp825lcd-diagnostics ─────────────────────────────────────────────────
  // Deliberately called from update(), NOT from setup() or handle_new_device().
  // This component is setup_priority::DATA (600) and WiFi is 250, so anything
  // logged during setup is emitted before the network exists and is
  // unreachable over the API / web_server log streams. Measured across three
  // restarts of device D while chasing the VID/PID line, which is emitted
  // exactly there and never arrived. update() runs in the main loop.
  run_hid_diagnostics_();

  // Check if protocol detection is needed
  if (!active_protocol_) {
    ESP_LOGI(TAG, log_messages::ATTEMPTING_DETECTION);
    if (detect_protocol()) {
      ESP_LOGI(TAG, log_messages::PROTOCOL_DETECTED);
      consecutive_failures_ = 0;
    } else {
      consecutive_failures_++;
      ESP_LOGW(TAG, log_messages::DETECTION_FAILED, consecutive_failures_);
      
      if (consecutive_failures_ > max_consecutive_failures_) {
        ESP_LOGE(TAG, log_messages::TOO_MANY_FAILURES);
        mark_failed();
      }
      return;
    }
  }
  
  // Normal data reading with active protocol
  if (read_ups_data()) {
    update_sensors();
    consecutive_failures_ = 0;
    last_successful_read_ = millis();
    
    // Check for timer updates (fast polling during countdowns)
    check_and_update_timers();
  } else {
    consecutive_failures_++;
    ESP_LOGW(TAG, log_messages::READ_FAILED, consecutive_failures_);
    
    if (consecutive_failures_ > max_consecutive_failures_) {
      ESP_LOGW(TAG, log_messages::RESETTING_PROTOCOL);
      active_protocol_.reset();  // Force protocol re-detection on next update
      consecutive_failures_ = 0;
    }
  }
}

// ── cp825lcd-diagnostics ────────────────────────────────────────────────────
// Everything here logs at ESP_LOGI on purpose: ESPHome bakes the log level into
// the build (ESPHOME_LOG_LEVEL), so ESP_LOGD statements are COMPILED OUT on a
// device running `logger: level: INFO` -- which device D is. A dump that is
// compiled out is indistinguishable from a request that failed.
void UpsHidComponent::run_hid_diagnostics_() {
  static const char *HEXC = "0123456789ABCDEF";

  diag_tick_++;

  // ---- (1) report-descriptor dump ----------------------------------------
  // ⛔ NOT one-shot, and NOT on the first tick. MEASURED 2026-08-22: the very
  // first update() after a reboot is UNREACHABLE over the network -- the
  // ESPHome API log SUBSCRIPTION is not live yet, so everything tick 1 emits
  // is dropped, exactly as the boot banner is (setup_priority::DATA vs WiFi).
  // Proof: two clean captures reconnected ~2 s after the OTA/restart
  // disconnect and BOTH began mid-stream at "SWEEP progress: next 0x10" --
  // tick 1's own progress line was missing too, so this is the subscription,
  // not the dump. Firing on ticks 2-6 gives five independent chances, all
  // after the subscription is provably live; it costs ~5 repeats of a short
  // hex dump and it is what makes the result CAPTURABLE rather than merely
  // emitted. Same family as the harness that discards a detector's findings.
  if (diag_tick_ >= 2 && diag_tick_ <= 6) {
    diag_descriptor_done_ = true;
    ESP_LOGI(TAG, "=== HID DIAGNOSTICS: VID=0x%04X PID=0x%04X ===",
             transport_->get_vendor_id(), transport_->get_product_id());

    std::vector<uint8_t> buf(512);
    size_t len = buf.size();
    esp_err_t ret = transport_->get_report_descriptor(buf.data(), &len, 2000);
    if (ret != ESP_OK || len == 0) {
      ESP_LOGE(TAG, "REPORT DESCRIPTOR READ FAILED: %s (len=%u)",
               esp_err_to_name(ret), (unsigned) len);
    } else {
      ESP_LOGI(TAG, "REPORT DESCRIPTOR: %u bytes", (unsigned) len);
      for (size_t i = 0; i < len; i += 16) {
        std::string line;
        for (size_t j = 0; j < 16 && (i + j) < len; j++) {
          uint8_t v = buf[i + j];
          line += HEXC[v >> 4];
          line += HEXC[v & 0x0F];
          line += ' ';
        }
        ESP_LOGI(TAG, "RD %03u: %s", (unsigned) i, line.c_str());
      }
      ESP_LOGI(TAG, "=== END REPORT DESCRIPTOR ===");
    }
  }

  // ---- (2) chunked report-ID sweep, FEATURE *and* INPUT -------------------
  // Chunked because the update loop is watchdog-exposed and the component
  // already warns past 500 ms. Refusals measured at ~9-10 ms, so 8 ids x 2
  // types is ~160 ms typical; the short per-request timeout bounds the bad
  // case, since a request that TIMES OUT costs 5 s at the normal setting.
  // Phase 2 runs only once the 0x00-0x5F sweep has finished, so the two never
  // share a tick and their results can never be confused for one another.
  if (diag_sweep_done_) { run_wlength_probe_(); return; }

  // NARROWED 0x5F -> 0x18 now that the wide sweep has already returned its clean
  // zero. Its job here is different and better: 0x01-0x18 is exactly the set the
  // wLength probe below tests, so this run is the CONTROL ARM for that 2x2 --
  // same ids, same session, wLength 64. "Arm C misses, arm A hits" is a far
  // stronger statement than "arm A hits" alone, and it costs 4 ticks not 12.
  const uint16_t SWEEP_LAST = 0x18;
  const uint16_t PER_TICK   = 8;
  const uint32_t TMO_MS     = 150;

  uint8_t buf[64];
  uint16_t done_this_tick = 0;
  while (diag_sweep_next_ <= SWEEP_LAST && done_this_tick < PER_TICK) {
    uint8_t id = (uint8_t) diag_sweep_next_;
    size_t l;

    l = sizeof(buf);
    if (transport_->hid_get_report(HID_REPORT_TYPE_FEATURE, id, buf, &l, TMO_MS) == ESP_OK && l > 0) {
      diag_found_++;
      ESP_LOGI(TAG, "SWEEP HIT  Feature 0x%02X  %u bytes", id, (unsigned) l);
    }
    l = sizeof(buf);
    if (transport_->hid_get_report(HID_REPORT_TYPE_INPUT, id, buf, &l, TMO_MS) == ESP_OK && l > 0) {
      diag_found_++;
      ESP_LOGI(TAG, "SWEEP HIT  Input   0x%02X  %u bytes", id, (unsigned) l);
    }

    diag_sweep_next_++;
    done_this_tick++;
  }

  if (diag_sweep_next_ > SWEEP_LAST) {
    diag_sweep_done_ = true;
    ESP_LOGI(TAG, "=== SWEEP COMPLETE: 0x00-0x%02X, %u hit(s) ===", SWEEP_LAST, diag_found_);
    if (diag_found_ == 0) {
      ESP_LOGI(TAG, "SWEEP: zero reports on the control pipe for BOTH types. "
                    "Standard requests work (enumeration succeeded), so this is "
                    "'no class GET_REPORT', not a dead control endpoint.");
    }
  } else {
    ESP_LOGI(TAG, "SWEEP progress: next 0x%02X (%u hit(s) so far)",
             (unsigned) diag_sweep_next_, diag_found_);
  }
}

// ── cp825lcd-diagnostics phase 2: the wLength 2x2 ───────────────────────────
// PRE-REGISTERED in docs/plans/ups-cp825lcd-no-hid-reports.md before it was run.
//
// The sweep asked all of 0x00-0x5F for BOTH report types and got a clean zero,
// which killed the "different report map" theory. The descriptor then showed
// this device declares 23 FEATURE reports at 0x01-0x18 with payloads of 1-6
// bytes -- IDs that OVERLAP the ones the driver already asks for. So the IDs
// were never the problem.
//
// The one field where we differ from NUT is wLength. NUT's libhid.c:131 sets
// rbuf->len[id] = replen[id] + 1 (payload plus the leading report-ID byte) and
// passes that; over-long asks are an opt-IN knob there (max_report_size). We
// hardcode 64 -- ten times the largest report on this device.
//
// hid_get_report() already derives wLength from the caller's *data_len
// (expected_len = min(*data_len, 64)), so this needs NO transport change: pass
// a small length and the setup packet follows. That also keeps the real read
// path untouched, which matters because A/B/C/E may work today precisely
// BECAUSE their firmware tolerates 64 -- a change there is fleet-wide.
//
// Two arms, so the result discriminates rather than merely yes/no:
//   A: declared payload + 1  -> a hit means exact-size requests are required
//   B: 8 for every id        -> a hit means any SMALL ask works; 64 is just too long
// Neither hits => hypothesis falsified, the setup packet is exonerated.
//
// GET only. Never SET: several CyberPower ids are commands when written (0x14
// is the self-test). Reading 0x14 is what the driver already does every 10 s.
void UpsHidComponent::run_wlength_probe_() {
  if (diag_wlen_done_) { run_raw_dump_(); return; }

  // Declared FEATURE payload per report id, parsed from this device's own
  // 383-byte report descriptor. 0x11 is not declared, hence absent.
  struct RepLen { uint8_t id; uint8_t payload; };
  static const RepLen DECLARED[] = {
    {0x01,1},{0x02,1},{0x03,1},{0x04,1},{0x05,1},{0x06,1},{0x07,6},{0x08,5},
    {0x09,1},{0x0A,1},{0x0B,1},{0x0C,1},{0x0D,1},{0x0E,1},{0x0F,1},{0x10,2},
    {0x12,1},{0x13,1},{0x14,1},{0x15,2},{0x16,2},{0x17,1},{0x18,2},
  };
  const uint16_t N = sizeof(DECLARED) / sizeof(DECLARED[0]);
  const uint16_t PER_TICK = 6;
  const uint32_t TMO_MS   = 150;

  if (diag_wlen_next_ == 0) {
    ESP_LOGI(TAG, "=== WLEN PROBE START: %u declared FEATURE ids, arm A=payload+1, arm B=8 ===", N);
  }

  uint8_t buf[64];
  uint16_t done_this_tick = 0;
  while (diag_wlen_next_ < N && done_this_tick < PER_TICK) {
    const RepLen &r = DECLARED[diag_wlen_next_];
    size_t l;

    // arm A -- NUT's default: payload + 1 for the leading report-id byte
    l = (size_t) r.payload + 1;
    if (transport_->hid_get_report(HID_REPORT_TYPE_FEATURE, r.id, buf, &l, TMO_MS) == ESP_OK && l > 0) {
      diag_wlen_hits_++;
      ESP_LOGI(TAG, "WLEN HIT  A  id=0x%02X wLength=%u -> %u bytes  first=0x%02X",
               r.id, (unsigned)(r.payload + 1), (unsigned) l, buf[0]);
    }

    // arm B -- a small constant, to separate "exact size" from "just not 64"
    l = 8;
    if (transport_->hid_get_report(HID_REPORT_TYPE_FEATURE, r.id, buf, &l, TMO_MS) == ESP_OK && l > 0) {
      diag_wlen_hits_++;
      ESP_LOGI(TAG, "WLEN HIT  B  id=0x%02X wLength=8 -> %u bytes  first=0x%02X",
               r.id, (unsigned) l, buf[0]);
    }

    diag_wlen_next_++;
    done_this_tick++;
  }

  if (diag_wlen_next_ >= N) {
    diag_wlen_done_ = true;
    ESP_LOGI(TAG, "=== WLEN PROBE COMPLETE: %u hit(s) across both arms ===", diag_wlen_hits_);
    if (diag_wlen_hits_ == 0) {
      ESP_LOGI(TAG, "WLEN: FALSIFIED -- exact-size and small-constant asks are refused too, "
                    "so wLength is not the discriminator and the setup packet is exonerated.");
    }
  } else {
    ESP_LOGI(TAG, "WLEN progress: %u/%u ids (%u hit(s) so far)",
             (unsigned) diag_wlen_next_, (unsigned) N, diag_wlen_hits_);
  }
}

// ── cp825lcd-diagnostics phase 3: RAW BYTES ─────────────────────────────────
// Why this exists: D reports Battery Voltage 20.2 V on a 12 V pack while its
// sibling E -- same 12 V / 450 W / 825 VA class, same parser -- reads 13.6 V
// correctly. So the divide-by-10 is NOT wrong for 12 V packs, and D's value is
// anomalous rather than mis-scaled. 20.2 implies a raw of 202 and 13.6 implies
// 136; 202 is not a truncation of 136, so it is not a lost high byte either.
//
// Everything above is INFERRED from the rendered sensor value. This dumps what
// the device actually returns, at two lengths, so the next step is grounded:
//   payload+1  what the driver now asks for
//   8          more than declared -- shows whether real data sits past the
//              declared width (D answers any small length, arm B proved that)
//
// The controls are in the same output and are the point: 0x09 (battery nominal)
// and 0x0E (input nominal) render CORRECTLY today, and sit next to 0x0A and
// 0x0F which do not -- same width, same declared unit exponent (6 and 7
// respectively). Whatever separates them is visible here or nowhere.
//
// GET only, never SET.
void UpsHidComponent::run_raw_dump_() {
  if (diag_raw_done_) return;
  static const char *HEXC = "0123456789ABCDEF";

  struct RepLen { uint8_t id; uint8_t payload; const char *what; };
  static const RepLen R[] = {
    {0x09,1,"battery nominal  CONTROL-ok"}, {0x0A,1,"battery actual   WRONG 20.2"},
    {0x0E,1,"input nominal    CONTROL-ok"}, {0x0F,1,"input actual     WRONG 230"},
    {0x10,2,"input transfer   NA"},         {0x12,1,"output voltage   NA"},
    {0x13,1,"load pct         ok"},         {0x15,2,"delay shutdown   WRONG -16246"},
    {0x07,6,"battery caps     ok"},         {0x08,5,"level+runtime    ok"},
  };
  const uint16_t N = sizeof(R) / sizeof(R[0]);
  if (diag_raw_next_ >= N) {
    diag_raw_done_ = true;
    ESP_LOGI(TAG, "=== RAW DUMP COMPLETE ===");
    return;
  }

  const RepLen &r = R[diag_raw_next_++];
  uint8_t buf[64];
  const uint16_t LENS[2] = { (uint16_t)(r.payload + 1), 8 };
  for (uint8_t k = 0; k < 2; k++) {
    size_t l = LENS[k];
    esp_err_t ret = transport_->hid_get_report(HID_REPORT_TYPE_FEATURE, r.id, buf, &l, 300);
    if (ret != ESP_OK || l == 0) {
      ESP_LOGI(TAG, "RAW 0x%02X wLen=%u -> REFUSED (%s)  [%s]",
               r.id, (unsigned) LENS[k], esp_err_to_name(ret), r.what);
      continue;
    }
    std::string hex;
    for (size_t j = 0; j < l && j < 16; j++) {
      hex += HEXC[buf[j] >> 4]; hex += HEXC[buf[j] & 0x0F]; hex += ' ';
    }
    // Also render the two interpretations the parsers actually use, so the
    // comparison does not need to be done by hand against a hex string.
    unsigned b1 = (l > 1) ? buf[1] : 0;
    unsigned le16 = (l > 2) ? (b1 | (buf[2] << 8)) : b1;
    ESP_LOGI(TAG, "RAW 0x%02X wLen=%u -> %u B: %s | byte1=%u (/10=%.1f) le16=%u (/10=%.1f)  [%s]",
             r.id, (unsigned) LENS[k], (unsigned) l, hex.c_str(),
             b1, b1 / 10.0f, le16, le16 / 10.0f, r.what);
  }
}

void UpsHidComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID Component:");
  ESP_LOGCONFIG(TAG, "  Simulation Mode: %s", simulation_mode_ ? status::YES : status::NO);
  
  if (transport_ && transport_->is_connected()) {
    ESP_LOGCONFIG(TAG, "  USB Vendor ID: 0x%04X", transport_->get_vendor_id());
    ESP_LOGCONFIG(TAG, "  USB Product ID: 0x%04X", transport_->get_product_id());
  }
  
  ESP_LOGCONFIG(TAG, "  Protocol Timeout: %u ms", protocol_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Protocol Selection: %s", protocol_selection_.c_str());
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", get_update_interval());

  if (transport_ && transport_->is_connected()) {
    ESP_LOGCONFIG(TAG, "  Status: %s", status::CONNECTED);
    if (active_protocol_) {
      ESP_LOGCONFIG(TAG, "  Active Protocol: %s", 
                   active_protocol_->get_protocol_name().c_str());
    } else {
      ESP_LOGCONFIG(TAG, "  Protocol Status: %s", status::DETECTION_PENDING);
    }
  } else {
    ESP_LOGCONFIG(TAG, "  Status: %s", status::DISCONNECTED);
  }

#ifdef USE_SENSOR
  ESP_LOGCONFIG(TAG, "  Registered Sensors: %zu", sensors_.size());
#endif
#ifdef USE_BINARY_SENSOR
  ESP_LOGCONFIG(TAG, "  Registered Binary Sensors: %zu", binary_sensors_.size());
#endif
#ifdef USE_TEXT_SENSOR
  ESP_LOGCONFIG(TAG, "  Registered Text Sensors: %zu", text_sensors_.size());
#endif
}

// Transport abstraction methods
esp_err_t UpsHidComponent::hid_get_report(uint8_t report_type, uint8_t report_id, 
                                         uint8_t* data, size_t* data_len, 
                                         uint32_t timeout_ms) {
  if (!transport_) {
    return ESP_ERR_INVALID_STATE;
  }
  return transport_->hid_get_report(report_type, report_id, data, data_len, timeout_ms);
}

esp_err_t UpsHidComponent::hid_set_report(uint8_t report_type, uint8_t report_id,
                                         const uint8_t* data, size_t data_len,
                                         uint32_t timeout_ms) {
  if (!transport_) {
    return ESP_ERR_INVALID_STATE;
  }
  return transport_->hid_set_report(report_type, report_id, data, data_len, timeout_ms);
}

esp_err_t UpsHidComponent::get_string_descriptor(uint8_t string_index, std::string& result) {
  if (!transport_) {
    return ESP_ERR_INVALID_STATE;
  }
  return transport_->get_string_descriptor(string_index, result);
}

bool UpsHidComponent::is_connected() const {
  return transport_ && transport_->is_connected();
}

uint16_t UpsHidComponent::get_vendor_id() const {
  return transport_ ? transport_->get_vendor_id() : defaults::AUTO_DETECT_VENDOR_ID;
}

uint16_t UpsHidComponent::get_product_id() const {
  return transport_ ? transport_->get_product_id() : defaults::AUTO_DETECT_PRODUCT_ID;
}

// Core implementation methods
bool UpsHidComponent::initialize_transport() {
  ESP_LOGD(TAG, "Initializing transport layer");
  
  // Create appropriate transport
  auto transport_type = simulation_mode_ ? 
    UsbTransportFactory::SIMULATION : 
    UsbTransportFactory::ESP32_HARDWARE;
    
  transport_ = UsbTransportFactory::create(transport_type, simulation_mode_);
  
  if (!transport_) {
    ESP_LOGE(TAG, "Failed to create transport instance");
    return false;
  }
  
  esp_err_t ret = transport_->initialize();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Transport initialization failed: %s", transport_->get_last_error().c_str());
    transport_.reset();
    return false;
  }
  
  connected_ = transport_->is_connected();
  
  ESP_LOGI(TAG, "Transport initialized successfully (VID=0x%04X, PID=0x%04X)", 
           transport_->get_vendor_id(), transport_->get_product_id());
  
  return true;
}

bool UpsHidComponent::detect_protocol() {
  if (!transport_ || !transport_->is_connected()) {
    ESP_LOGE(TAG, "Cannot detect protocol - transport not connected");
    return false;
  }
  
  uint16_t vendor_id = transport_->get_vendor_id();
  
  if (protocol_selection_ == "auto") {
    // Automatic protocol detection based on vendor ID
    ESP_LOGD(TAG, "Auto-detecting protocol for vendor 0x%04X using factory", vendor_id);
    active_protocol_ = ProtocolFactory::create_for_vendor(vendor_id, this);
  } else {
    // Manual protocol selection via factory
    ESP_LOGD(TAG, "Using manually selected protocol: %s", protocol_selection_.c_str());
    active_protocol_ = ProtocolFactory::create_by_name(protocol_selection_, this);
  }
  
  if (!active_protocol_) {
    ESP_LOGE(TAG, "Failed to create protocol (selection: %s, vendor: 0x%04X)", 
             protocol_selection_.c_str(), vendor_id);
    return false;
  }
  
  ESP_LOGI(TAG, "Successfully created protocol: %s", active_protocol_->get_protocol_name().c_str());
  
  // Initialize the protocol (detection already done by factory)
  if (!active_protocol_->initialize()) {
    ESP_LOGE(TAG, "Protocol initialization failed");
    active_protocol_.reset();
    return false;
  }
  
  ESP_LOGI(TAG, "Protocol initialized: %s", 
           active_protocol_->get_protocol_name().c_str());
  
  // Set the detected protocol in ups_data_ after successful detection
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ups_data_.device.detected_protocol = active_protocol_->get_protocol_type();
  }
  
  return true;
}

bool UpsHidComponent::read_ups_data() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for reading data");
    return false;
  }
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Preserve the detected protocol before reset
  DeviceInfo::DetectedProtocol current_protocol = ups_data_.device.detected_protocol;
  
  // Reset data before reading
  ups_data_.reset();
  
  // Restore the detected protocol after reset
  ups_data_.device.detected_protocol = current_protocol;
  
  // Read data through protocol
  bool success = active_protocol_->read_data(ups_data_);
  
  if (success) {
    ESP_LOGV(TAG, "Successfully read UPS data");
  } else {
    ESP_LOGW(TAG, "Failed to read UPS data via protocol");
  }
  
  return success;
}

void UpsHidComponent::update_sensors() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Check if any sensors are registered - if not, skip sensor updates
  size_t total_sensors = 0;
#ifdef USE_SENSOR
  total_sensors += sensors_.size();
#endif
#ifdef USE_BINARY_SENSOR
  total_sensors += binary_sensors_.size();
#endif
#ifdef USE_TEXT_SENSOR
  total_sensors += text_sensors_.size();
#endif
  
  if (total_sensors == 0) {
    // Data provider mode - no sensor entities, just providing data for direct access
    ESP_LOGVV(TAG, "Data provider mode: no sensors registered, data available via direct access methods");
    return;
  }
  
  ESP_LOGVV(TAG, "Updating %zu registered sensor entities", total_sensors);
  
  // Update all registered sensors with current data
#ifdef USE_SENSOR  
  for (auto& sensor_pair : sensors_) {
    const std::string& type = sensor_pair.first;
    sensor::Sensor* sensor = sensor_pair.second;
    
    // Extract appropriate value based on sensor type
    float value = NAN;
    
    if (type == sensor_type::BATTERY_LEVEL && ups_data_.battery.is_valid()) {
      value = ups_data_.battery.level;
    } else if (type == sensor_type::BATTERY_VOLTAGE && !std::isnan(ups_data_.battery.voltage)) {
      value = ups_data_.battery.voltage;
    } else if (type == sensor_type::BATTERY_VOLTAGE_NOMINAL && !std::isnan(ups_data_.battery.voltage_nominal)) {
      value = ups_data_.battery.voltage_nominal;
    } else if (type == sensor_type::RUNTIME && !std::isnan(ups_data_.battery.runtime_minutes)) {
      value = ups_data_.battery.runtime_minutes;
    } else if (type == sensor_type::INPUT_VOLTAGE && !std::isnan(ups_data_.power.input_voltage)) {
      value = ups_data_.power.input_voltage;
    } else if (type == sensor_type::INPUT_VOLTAGE_NOMINAL && !std::isnan(ups_data_.power.input_voltage_nominal)) {
      value = ups_data_.power.input_voltage_nominal;
    } else if (type == sensor_type::OUTPUT_VOLTAGE && !std::isnan(ups_data_.power.output_voltage)) {
      value = ups_data_.power.output_voltage;
    } else if (type == sensor_type::LOAD_PERCENT && !std::isnan(ups_data_.power.load_percent)) {
      value = ups_data_.power.load_percent;
    } else if (type == sensor_type::FREQUENCY && !std::isnan(ups_data_.power.frequency)) {
      value = ups_data_.power.frequency;
    } else if (type == sensor_type::INPUT_TRANSFER_LOW && !std::isnan(ups_data_.power.input_transfer_low)) {
      value = ups_data_.power.input_transfer_low;
    } else if (type == sensor_type::INPUT_TRANSFER_HIGH && !std::isnan(ups_data_.power.input_transfer_high)) {
      value = ups_data_.power.input_transfer_high;
    } else if (type == sensor_type::BATTERY_RUNTIME_LOW && !std::isnan(ups_data_.battery.runtime_low)) {
      value = ups_data_.battery.runtime_low;
    } else if (type == sensor_type::UPS_REALPOWER_NOMINAL && !std::isnan(ups_data_.power.realpower_nominal)) {
      value = ups_data_.power.realpower_nominal;
    } else if (type == sensor_type::UPS_DELAY_SHUTDOWN && !std::isnan(ups_data_.config.delay_shutdown)) {
      value = ups_data_.config.delay_shutdown;
    } else if (type == sensor_type::UPS_DELAY_START && !std::isnan(ups_data_.config.delay_start)) {
      value = ups_data_.config.delay_start;
    } else if (type == sensor_type::UPS_DELAY_REBOOT && !std::isnan(ups_data_.config.delay_reboot)) {
      value = ups_data_.config.delay_reboot;
    } else if (type == sensor_type::UPS_TIMER_REBOOT && ups_data_.test.timer_reboot != -1) {
      value = ups_data_.test.timer_reboot;
    } else if (type == sensor_type::UPS_TIMER_SHUTDOWN && ups_data_.test.timer_shutdown != -1) {
      value = ups_data_.test.timer_shutdown;
    } else if (type == sensor_type::UPS_TIMER_START && ups_data_.test.timer_start != -1) {
      value = ups_data_.test.timer_start;
    }
    
    if (!std::isnan(value)) {
      sensor->publish_state(value);
    }
  }
#endif
  
  // Update binary sensors
#ifdef USE_BINARY_SENSOR
  for (auto& sensor_pair : binary_sensors_) {
    const std::string& type = sensor_pair.first;
    binary_sensor::BinarySensor* sensor = sensor_pair.second;
    
    bool state = false;
    
    if (type == binary_sensor_type::ONLINE && ups_data_.power.input_voltage_valid()) {
      state = true;
    } else if (type == binary_sensor_type::ON_BATTERY && ups_data_.power.input_voltage_valid()) {
      state = false; // Opposite of online
    } else if (type == binary_sensor_type::LOW_BATTERY) {
      state = ups_data_.battery.is_low();
    }
    
    sensor->publish_state(state);
  }
#endif
  
  // Update text sensors
#ifdef USE_TEXT_SENSOR  
  for (auto& sensor_pair : text_sensors_) {
    const std::string& type = sensor_pair.first;
    text_sensor::TextSensor* sensor = sensor_pair.second;
    
    std::string value = "";
    
    if (type == text_sensor_type::MODEL && !ups_data_.device.model.empty()) {
      value = ups_data_.device.model;
    } else if (type == text_sensor_type::MANUFACTURER && !ups_data_.device.manufacturer.empty()) {
      value = ups_data_.device.manufacturer;
    } else if (type == text_sensor_type::SERIAL_NUMBER && !ups_data_.device.serial_number.empty()) {
      value = ups_data_.device.serial_number;
    } else if (type == text_sensor_type::FIRMWARE_VERSION && !ups_data_.device.firmware_version.empty()) {
      value = ups_data_.device.firmware_version;
    } else if (type == text_sensor_type::BATTERY_STATUS && !ups_data_.battery.status.empty()) {
      value = ups_data_.battery.status;
    } else if (type == text_sensor_type::UPS_TEST_RESULT && !ups_data_.test.ups_test_result.empty()) {
      value = ups_data_.test.ups_test_result;
    } else if (type == text_sensor_type::UPS_BEEPER_STATUS && !ups_data_.config.beeper_status.empty()) {
      value = ups_data_.config.beeper_status;
    } else if (type == text_sensor_type::INPUT_SENSITIVITY && !ups_data_.config.input_sensitivity.empty()) {
      value = ups_data_.config.input_sensitivity;
    } else if (type == text_sensor_type::STATUS && !ups_data_.power.status.empty()) {
      value = ups_data_.power.status;
    } else if (type == text_sensor_type::PROTOCOL) {
      value = get_protocol_name();
    } else if (type == text_sensor_type::BATTERY_MFR_DATE && !ups_data_.battery.mfr_date.empty()) {
      value = ups_data_.battery.mfr_date;
    } else if (type == text_sensor_type::UPS_MFR_DATE && !ups_data_.device.mfr_date.empty()) {
      value = ups_data_.device.mfr_date;
    } else if (type == text_sensor_type::BATTERY_TYPE && !ups_data_.battery.type.empty()) {
      value = ups_data_.battery.type;
    } else if (type == text_sensor_type::UPS_FIRMWARE_AUX && !ups_data_.device.firmware_aux.empty()) {
      value = ups_data_.device.firmware_aux;
    }
    
    if (!value.empty()) {
      sensor->publish_state(value);
    }
  }
#endif
  
  // Update delay number components
  for (auto* delay_number : delay_numbers_) {
    if (delay_number != nullptr) {
      // Determine which delay value to use based on the delay type
      // This will be handled by the number component itself
      // We just need to trigger an update with the current config values
      // The number component will query the appropriate value
    }
  }
  
  // Log sensor counts (conditional on platform availability)
#ifdef USE_SENSOR
  size_t sensor_count = sensors_.size();
#else
  size_t sensor_count = 0;
#endif
#ifdef USE_BINARY_SENSOR
  size_t binary_sensor_count = binary_sensors_.size(); 
#else
  size_t binary_sensor_count = 0;
#endif
#ifdef USE_TEXT_SENSOR
  size_t text_sensor_count = text_sensors_.size();
#else
  size_t text_sensor_count = 0;
#endif
  
  ESP_LOGV(TAG, "Updated %zu sensors, %zu binary sensors, %zu text sensors", 
           sensor_count, binary_sensor_count, text_sensor_count);
}

// Sensor registration methods (conditional on platform availability)
#ifdef USE_SENSOR
void UpsHidComponent::register_sensor(sensor::Sensor *sens, const std::string &type) {
  sensors_[type] = sens;
  ESP_LOGD(TAG, "Registered sensor: %s", type.c_str());
}
#endif

#ifdef USE_BINARY_SENSOR
void UpsHidComponent::register_binary_sensor(binary_sensor::BinarySensor *sens, const std::string &type) {
  binary_sensors_[type] = sens;
  ESP_LOGD(TAG, "Registered binary sensor: %s", type.c_str());
}
#endif

#ifdef USE_TEXT_SENSOR
void UpsHidComponent::register_text_sensor(text_sensor::TextSensor *sens, const std::string &type) {
  text_sensors_[type] = sens;
  ESP_LOGD(TAG, "Registered text sensor: %s", type.c_str());
}
#endif

void UpsHidComponent::register_delay_number(UpsDelayNumber *number) {
  delay_numbers_.push_back(number);
  ESP_LOGD(TAG, "Registered delay number component");
}

// Test control methods
bool UpsHidComponent::start_battery_test_quick() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for battery test");
    return false;
  }
  return active_protocol_->start_battery_test_quick();
}

bool UpsHidComponent::start_battery_test_deep() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for battery test");
    return false;
  }
  return active_protocol_->start_battery_test_deep();
}

bool UpsHidComponent::stop_battery_test() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for battery test");
    return false;
  }
  return active_protocol_->stop_battery_test();
}

bool UpsHidComponent::start_ups_test() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for UPS test");
    return false;
  }
  return active_protocol_->start_ups_test();
}

bool UpsHidComponent::stop_ups_test() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for UPS test");
    return false;
  }
  return active_protocol_->stop_ups_test();
}

// Beeper control methods
bool UpsHidComponent::beeper_enable() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for beeper control");
    return false;
  }
  return active_protocol_->beeper_enable();
}

bool UpsHidComponent::beeper_disable() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for beeper control");
    return false;
  }
  return active_protocol_->beeper_disable();
}

bool UpsHidComponent::beeper_mute() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for beeper control");
    return false;
  }
  return active_protocol_->beeper_mute();
}

bool UpsHidComponent::beeper_test() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for beeper control");
    return false;
  }
  return active_protocol_->beeper_test();
}

// Delay configuration methods
bool UpsHidComponent::set_shutdown_delay(int seconds) {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for delay configuration");
    return false;
  }
  return active_protocol_->set_shutdown_delay(seconds);
}

bool UpsHidComponent::set_start_delay(int seconds) {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for delay configuration");
    return false;
  }
  return active_protocol_->set_start_delay(seconds);
}

bool UpsHidComponent::set_reboot_delay(int seconds) {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for delay configuration");
    return false;
  }
  return active_protocol_->set_reboot_delay(seconds);
}

// Additional protocol access method
std::string UpsHidComponent::get_protocol_name() const {
  if (active_protocol_) {
    return active_protocol_->get_protocol_name();
  }
  return protocol::NONE;
}


// Error rate limiting helpers
bool UpsHidComponent::should_log_error(ErrorRateLimit& limiter) {
  uint32_t now = millis();
  
  if (limiter.error_count < ErrorRateLimit::MAX_BURST) {
    limiter.error_count++;
    limiter.last_error_time = now;
    return true;
  }
  
  if (now - limiter.last_error_time > ErrorRateLimit::RATE_LIMIT_MS) {
    limiter.error_count = 1;
    limiter.suppressed_count = 0;
    limiter.last_error_time = now;
    return true;
  }
  
  limiter.suppressed_count++;
  return false;
}

void UpsHidComponent::log_suppressed_errors(ErrorRateLimit& limiter) {
  if (limiter.suppressed_count > 0) {
    ESP_LOGW(TAG, "Suppressed %u similar errors in the last %u ms", 
             limiter.suppressed_count, ErrorRateLimit::RATE_LIMIT_MS);
    limiter.suppressed_count = 0;
  }
}

void UpsHidComponent::cleanup() {
  if (transport_) {
    transport_->deinitialize();
    transport_.reset();
  }
  
  active_protocol_.reset();
  connected_ = false;
  
  ESP_LOGD(TAG, "Component cleanup completed");
}

// Timer polling implementation
void UpsHidComponent::check_and_update_timers() {
  if (!active_protocol_) return;
  
  uint32_t now = millis();
  
  // Check if we need to poll timers
  bool should_poll_timers = false;
  
  if (fast_polling_mode_) {
    // In fast polling mode, check every 2 seconds
    if (now - last_timer_poll_ >= FAST_POLL_INTERVAL_MS) {
      should_poll_timers = true;
    }
  } else {
    // Check if any timers might be active (less frequent check)
    if (now - last_timer_poll_ >= get_update_interval()) {
      should_poll_timers = true;
    }
  }
  
  if (should_poll_timers) {
    last_timer_poll_ = now;
    
    // Try to read timer data
    UpsData timer_data = ups_data_;  // Copy current data
    if (active_protocol_->read_timer_data(timer_data)) {
      // Update only timer-related fields
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ups_data_.test.timer_shutdown = timer_data.test.timer_shutdown;
        ups_data_.test.timer_start = timer_data.test.timer_start;
        ups_data_.test.timer_reboot = timer_data.test.timer_reboot;
      }
      
      // Update fast polling mode based on timer activity
      bool timers_active = has_active_timers();
      if (timers_active != fast_polling_mode_) {
        set_fast_polling_mode(timers_active);
      }
      
      // Update timer sensors immediately when values change
      update_sensors();
    }
  }
}

bool UpsHidComponent::has_active_timers() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return (ups_data_.test.timer_shutdown > 0 || 
          ups_data_.test.timer_start > 0 || 
          ups_data_.test.timer_reboot > 0);
}

void UpsHidComponent::set_fast_polling_mode(bool enable) {
  if (enable != fast_polling_mode_) {
    fast_polling_mode_ = enable;
    if (enable) {
      ESP_LOGI(TAG, "Enabled fast polling for timer countdown");
    } else {
      ESP_LOGI(TAG, "Disabled fast polling, returning to normal interval");
    }
  }
}

// Convenient state getters for lambda expressions (no sensor entities required)
bool UpsHidComponent::is_online() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // UPS is online when input voltage is valid (same logic as binary sensor update)
  return ups_data_.power.input_voltage_valid();
}

bool UpsHidComponent::is_on_battery() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // UPS is on battery when NOT online (opposite of online state)
  return !ups_data_.power.input_voltage_valid();
}

bool UpsHidComponent::is_low_battery() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // Use battery's built-in low battery detection
  return ups_data_.battery.is_low();
}

bool UpsHidComponent::is_charging() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // Charging when online AND battery level is not 100%
  return ups_data_.power.input_voltage_valid() && 
         ups_data_.battery.is_valid() && 
         !std::isnan(ups_data_.battery.level) && 
         ups_data_.battery.level < 100.0f;
}

bool UpsHidComponent::has_fault() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // Check for various fault conditions based on available data
  return ups_data_.power.is_input_out_of_range() || 
         (!ups_data_.power.is_valid() && !ups_data_.battery.is_valid());
}

bool UpsHidComponent::is_overloaded() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // Use power's built-in overload detection
  return ups_data_.power.is_overloaded();
}

float UpsHidComponent::get_battery_level() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.battery.is_valid() ? ups_data_.battery.level : NAN;
}

float UpsHidComponent::get_input_voltage() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.power.input_voltage;
}

float UpsHidComponent::get_output_voltage() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.power.output_voltage;
}

float UpsHidComponent::get_load_percent() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.power.load_percent;
}

float UpsHidComponent::get_runtime_minutes() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.battery.runtime_minutes;
}


} // namespace ups_hid
} // namespace esphome