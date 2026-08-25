#!/usr/bin/env python3
"""Static checks on components/ups_hid/sensor.py's per-type defaults.

Parsed by AST rather than imported, so this needs neither ESPHome nor pytest.
Run it either way:

    python3 tests/test_sensor_defaults.py
    pytest tests/test_sensor_defaults.py

⚠ These are STRUCTURAL checks and cannot prove the component compiles. The
state_class default shipped broken once while py_compile, an AST check, a logic
harness and `esphome config` (rc=0) all passed -- only a real `esphome compile`
caught it (see test_state_class_default_is_mapped_to_enum). The compilation
tests in test_esphome_configurations.py remain the real gate.
"""

import ast
import sys
from pathlib import Path

SENSOR_PY = Path(__file__).parent.parent / "components" / "ups_hid" / "sensor.py"

# The three ups_timer_* countdowns carry a -1 sentinel when inactive, so a
# long-term-statistics mean over them is wrong rather than merely trivial.
TIMERS_WITHOUT_STATE_CLASS = {
    "ups_timer_reboot",
    "ups_timer_shutdown",
    "ups_timer_start",
}


def _tree():
    return ast.parse(SENSOR_PY.read_text())


def _sensor_types():
    node = next(
        n for n in _tree().body
        if isinstance(n, ast.Assign) and n.targets[0].id == "SENSOR_TYPES"
    )
    return {
        k.value: {
            kk.value: (vv.id if isinstance(vv, ast.Name) else getattr(vv, "value", None))
            for kk, vv in zip(v.keys, v.values)
        }
        for k, v in zip(node.value.keys, node.value.values)
    }


def _state_class_guard():
    """The `if ...: config[CONF_STATE_CLASS] = ...` block inside to_code()."""
    fn = next(
        n for n in _tree().body
        if isinstance(n, ast.AsyncFunctionDef) and n.name == "to_code"
    )
    found = [
        n for n in ast.walk(fn)
        if isinstance(n, ast.If)
        and isinstance(n.test, ast.BoolOp)
        and isinstance(n.body[0], ast.Assign)
        and isinstance(n.body[0].targets[0], ast.Subscript)
        and getattr(n.body[0].targets[0].slice, "id", None) == "CONF_STATE_CLASS"
    ]
    assert len(found) == 1, f"expected 1 state_class assignment, found {len(found)}"
    return found[0]


def test_every_non_timer_type_has_a_state_class_default():
    """Without state_class, Home Assistant records no long-term statistics."""
    types = _sensor_types()
    missing = {k for k, v in types.items() if "state_class" not in v}
    assert missing == TIMERS_WITHOUT_STATE_CLASS, f"unexpected exclusions: {sorted(missing)}"


def test_state_class_defaults_are_measurement():
    types = _sensor_types()
    values = {v["state_class"] for v in types.values() if "state_class" in v}
    assert values == {"STATE_CLASS_MEASUREMENT"}, values


def test_yaml_state_class_still_wins():
    """The default must not clobber an explicit YAML value."""
    left = _state_class_guard().test.values[0]
    assert isinstance(left, ast.Compare) and isinstance(left.ops[0], ast.NotIn), (
        "guard is not `CONF_STATE_CLASS not in config` -- an explicit YAML "
        "state_class would be silently overwritten by the default"
    )
    assert getattr(left.left, "id", None) == "CONF_STATE_CLASS"
    assert getattr(left.comparators[0], "id", None) == "config"


def test_state_class_default_is_mapped_to_enum():
    """Sensor::set_state_class() takes a StateClass ENUM, not a string.

    device_class and unit_of_measurement ARE strings in C++, so injecting the
    raw string works for them. state_class is different: the YAML path reaches
    the enum through the schema's cv.enum(STATE_CLASSES), which writing into
    config after validation bypasses. A bare string emits

        set_state_class("measurement");

    which does not compile. This regression shipped once.
    """
    value = _state_class_guard().body[0].value
    assert isinstance(value, ast.Call), (
        "state_class default is assigned raw -- it must go through "
        "validate_state_class() or the generated C++ will not compile"
    )
    name = getattr(value.func, "attr", None) or getattr(value.func, "id", None)
    assert name == "validate_state_class", name


def test_config_schema_installs_no_defaults_that_kill_to_code():
    """sensor_schema() keywords make a key ALWAYS present, killing to_code().

    Every per-type default below is applied by an `if <key> not in config`
    guard. Passing unit_of_measurement=/device_class=/accuracy_decimals=/
    state_class= to sensor_schema() installs cv.Optional(key, default=...),
    so the key survives validation and that guard is permanently False --
    the per-type value is silently never applied.

    This is not hypothetical: `accuracy_decimals=1` did exactly that, and all
    24 per-type precisions were dead. Measured in generated code: battery_level
    emitted set_accuracy_decimals(1) while declaring 0.
    """
    tree = _tree()
    schema = next(
        n for n in tree.body
        if isinstance(n, ast.Assign) and n.targets[0].id == "CONFIG_SCHEMA"
    )
    calls = [n for n in ast.walk(schema) if isinstance(n, ast.Call)
             and getattr(n.func, "attr", None) == "sensor_schema"]
    assert len(calls) == 1, f"expected 1 sensor_schema() call, found {len(calls)}"
    passed = {kw.arg for kw in calls[0].keywords}
    forbidden = passed & {
        "unit_of_measurement", "device_class", "accuracy_decimals", "state_class",
    }
    assert not forbidden, (
        f"sensor_schema() was passed {sorted(forbidden)} -- each installs a "
        "schema default, which makes the matching `not in config` guard in "
        "to_code() permanently dead and silently discards the per-type value"
    )


def test_every_type_declares_accuracy_decimals():
    """The per-type precisions must be a total function over SENSOR_TYPES.

    CONF_TYPE is Required and one_of(SENSOR_TYPES), so with no schema default
    a type missing accuracy_decimals would get none at all.
    """
    types = _sensor_types()
    missing = [k for k, v in types.items() if "accuracy_decimals" not in v]
    assert not missing, missing


def test_state_class_constants_are_imported():
    imported = {
        alias.name
        for node in _tree().body
        if isinstance(node, ast.ImportFrom) and node.module == "esphome.const"
        for alias in node.names
    }
    assert {"CONF_STATE_CLASS", "STATE_CLASS_MEASUREMENT"} <= imported, imported


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failures = 0
    for t in tests:
        try:
            t()
            print(f"PASS  {t.__name__}")
        except AssertionError as exc:
            failures += 1
            print(f"FAIL  {t.__name__}: {exc}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    sys.exit(1 if failures else 0)
