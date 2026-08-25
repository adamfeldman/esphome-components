import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ACCURACY_DECIMALS,
    CONF_DEVICE_CLASS,
    CONF_STATE_CLASS,
    CONF_TYPE,
    CONF_UNIT_OF_MEASUREMENT,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_EMPTY,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_MINUTE,
    UNIT_HERTZ,
    UNIT_WATT,
    UNIT_SECOND,
    UNIT_VOLT_AMPS,
    DEVICE_CLASS_APPARENT_POWER,
    STATE_CLASS_MEASUREMENT,
)


from . import ups_hid_ns, UpsHidComponent, CONF_UPS_HID_ID

DEPENDENCIES = ["ups_hid"]

UpsHidSensor = ups_hid_ns.class_("UpsHidSensor", sensor.Sensor, cg.Component)

SENSOR_TYPES = {
    "battery_level": {
        "unit": UNIT_PERCENT,
        "device_class": DEVICE_CLASS_BATTERY,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "input_voltage": {
        "unit": UNIT_VOLT,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "output_voltage": {
        "unit": UNIT_VOLT,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "load_percent": {
        "unit": UNIT_PERCENT,
        "device_class": DEVICE_CLASS_POWER_FACTOR,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "runtime": {
        "unit": UNIT_MINUTE,
        "device_class": DEVICE_CLASS_DURATION,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "frequency": {
        "unit": UNIT_HERTZ,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "battery_voltage": {
        "unit": UNIT_VOLT,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "battery_voltage_nominal": {
        "unit": UNIT_VOLT,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "input_voltage_nominal": {
        "unit": UNIT_VOLT,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "input_transfer_low": {
        "unit": UNIT_VOLT,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "input_transfer_high": {
        "unit": UNIT_VOLT,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "ups_realpower_nominal": {
        "unit": UNIT_WATT,
        "device_class": DEVICE_CLASS_POWER,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    # MEASURED output power, read from the device -- NOT the "UPS Load Power"
    # template sensor in the maintainer's extended_sensors.yaml, which estimates
    # realpower_nominal x load% (with a 700 W fallback). Both can coexist; the
    # names are deliberately distinct so a dashboard cannot confuse them.
    #
    # ⚠ Only devices whose report descriptor declares the usage will ever publish
    # these. On this fleet that is 4 of 5 -- the CP825LCD declares neither, and
    # its sensors would sit at unknown forever, so do not declare them there.
    "ups_realpower": {
        "unit": UNIT_WATT,
        "device_class": DEVICE_CLASS_POWER,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "ups_apparent_power": {
        "unit": UNIT_VOLT_AMPS,
        "device_class": DEVICE_CLASS_APPARENT_POWER,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "ups_apparent_power_nominal": {
        "unit": UNIT_VOLT_AMPS,
        "device_class": DEVICE_CLASS_APPARENT_POWER,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "ups_delay_shutdown": {
        "unit": UNIT_SECOND,
        "device_class": DEVICE_CLASS_DURATION,
        "accuracy_decimals": 0,
    },
    "ups_delay_start": {
        "unit": UNIT_SECOND,
        "device_class": DEVICE_CLASS_DURATION,
        "accuracy_decimals": 0,
    },
    "ups_delay_reboot": {
        "unit": UNIT_SECOND,
        "device_class": DEVICE_CLASS_DURATION,
        "accuracy_decimals": 0,
    },
    # Additional missing sensor types from NUT analysis
    "battery_charge_low": {
        "unit": UNIT_PERCENT,
        "device_class": DEVICE_CLASS_BATTERY,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "battery_charge_warning": {
        "unit": UNIT_PERCENT,
        "device_class": DEVICE_CLASS_BATTERY,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    "battery_runtime_low": {
        "unit": UNIT_MINUTE,
        "device_class": DEVICE_CLASS_DURATION,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
    },
    # ⚠ NO state_class on the three ups_timer_* countdowns OR the three ups_delay_*
    # settings, deliberately. All six carry a -1 SENTINEL meaning "not set"/"inactive"
    # -- it is the documented default of the fields themselves (data_config.h:
    # `int16_t delay_shutdown{-1}  // -1 = not set`) -- so a long-term-statistics mean
    # over them is not merely trivial, it is wrong: it averages a sentinel with seconds.
    #
    # ⛔ The ups_delay_* trio was EXCLUDED ON 2026-08-25 AFTER SHIPPING WITH state_class,
    # and the mistake is worth keeping visible: the original comment here cited
    # `ups_delay_reboot = -1.0` as the EVIDENCE for excluding the ups_timer_* trio, and
    # then left state_class on ups_delay_reboot itself. The evidence and the exclusion
    # disagreed two lines apart. Confirmed live before fixing: ups_delay_reboot reads
    # -1.0 on BOTH device A and device D.
    #
    # Measured on the CyberPower fleet, and each fact independently justifies exclusion:
    #   * delay_reboot is NEVER assigned by the CyberPower protocol at all (only APC and
    #     generic populate it), so on these five devices it is permanently the -1 default;
    #   * device D reports delay_shutdown = -16246 (0xC08A) -- its 2007-era firmware does
    #     not implement DelayBeforeShutdown. See the plausibility guard in
    #     protocol_cyberpower.cpp, which now rejects it.
    # These are CONFIGURATION values, not measurements. Every other type here is a real
    # quantity, hence measurement.
    "ups_timer_reboot": {
        "unit": UNIT_SECOND,
        "device_class": DEVICE_CLASS_DURATION,
        "accuracy_decimals": 0,
    },
    "ups_timer_shutdown": {
        "unit": UNIT_SECOND,
        "device_class": DEVICE_CLASS_DURATION,
        "accuracy_decimals": 0,
    },
    "ups_timer_start": {
        "unit": UNIT_SECOND,
        "device_class": DEVICE_CLASS_DURATION,
        "accuracy_decimals": 0,
    },
}


# ⛔ DO NOT pass accuracy_decimals= (or device_class=, unit_of_measurement=,
# state_class=) to sensor_schema(). Any of those installs
# cv.Optional(key, default=...), which makes the key ALWAYS present after
# validation -- and every `if <key> not in config` block in to_code() below is
# then permanently dead, silently. That is exactly what `accuracy_decimals=1`
# did: all 24 per-type precisions were ignored and every sensor rendered at one
# decimal (a battery percentage as "87.0%"). CONF_TYPE is Required and
# one_of(SENSOR_TYPES), and every entry declares accuracy_decimals, so the
# per-type values below are a total function -- no sensor can fall through.
CONFIG_SCHEMA = sensor.sensor_schema(
    UpsHidSensor,
).extend(
    {
        cv.GenerateID(CONF_UPS_HID_ID): cv.use_id(UpsHidComponent),
        cv.Required(CONF_TYPE): cv.one_of(*SENSOR_TYPES, lower=True),
    }
)


async def to_code(config):
    sensor_type = config[CONF_TYPE]

    # Fill in the per-type defaults *before* the entity is created. ESPHome
    # bakes these into App.register_sensor() and no longer exposes a
    # set_device_class() setter on the C++ side, so applying them afterwards
    # fails to compile.
    if sensor_type in SENSOR_TYPES:
        sensor_config = SENSOR_TYPES[sensor_type]

        if CONF_UNIT_OF_MEASUREMENT not in config and "unit" in sensor_config:
            config[CONF_UNIT_OF_MEASUREMENT] = sensor_config["unit"]

        if CONF_DEVICE_CLASS not in config and "device_class" in sensor_config:
            config[CONF_DEVICE_CLASS] = sensor_config["device_class"]

        if CONF_ACCURACY_DECIMALS not in config and "accuracy_decimals" in sensor_config:
            config[CONF_ACCURACY_DECIMALS] = sensor_config["accuracy_decimals"]

        # ⚠ state_class is NOT like its three siblings above. device_class and
        # unit_of_measurement are plain strings in C++, so injecting the raw
        # string works. Sensor::set_state_class() takes an ENUM -- the YAML path
        # gets there via the schema's cv.enum(STATE_CLASSES), which we bypass by
        # writing into config after validation. Injecting the bare string emits
        # `set_state_class("measurement")` and FAILS TO COMPILE. Run it through
        # the same validator the schema uses, so the value is identical to YAML's.
        if CONF_STATE_CLASS not in config and "state_class" in sensor_config:
            config[CONF_STATE_CLASS] = sensor.validate_state_class(
                sensor_config["state_class"]
            )

    parent = await cg.get_variable(config[CONF_UPS_HID_ID])
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    cg.add(var.set_sensor_type(sensor_type))
    cg.add(parent.register_sensor(var, sensor_type))
