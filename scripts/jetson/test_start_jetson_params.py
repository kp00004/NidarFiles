"""start_jetson.sh must hand every float-typed ROS parameter to ros2 as a
double. ros2 reads `-p yaw_offset_deg:=270` as an INTEGER and a node that
declares the parameter as a double then refuses it and exits -- found on the
Jetson 2026-10-09 (lidar_node crashed on the default yaw "0"). No hardware
or ROS needed: this reads the node sources and runs the script's own
ros_double function in bash.

Run from the NidarFiles root:  python -m pytest scripts -q
"""
import os
import re
import shutil
import subprocess

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(ROOT, "scripts", "jetson", "start_jetson.sh")
NODES = [
    os.path.join(ROOT, "onboard-autonomy", "nidar_autonomy", "nidar_autonomy", name)
    for name in ("radio_command_node.py", "lidar_node.py")
]
# A ROS (YAML) double literal as rcl parses -p values: digits with a decimal point.
ROS_DOUBLE = re.compile(r"^[+-]?\d+\.\d+$")


def _read(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def float_params():
    names = set()
    for path in NODES:
        names |= set(re.findall(r'float\(\s*self\.declare_parameter\(\s*"(\w+)"', _read(path)))
    return names


def passed_params():
    """(param, shell variable) for every `-p "param:=$VAR"` in the script."""
    return re.findall(r'-p "(\w+):=\$(\w+)"', _read(SCRIPT))


def test_the_float_parameters_are_found():
    assert {"telemetry_rate_hz", "lidar_rate_hz", "yaw_offset_deg"} <= float_params()


def test_every_float_parameter_passed_goes_through_ros_double():
    script = _read(SCRIPT)
    passed = [(p, v) for p, v in passed_params() if p in float_params()]
    assert {p for p, _ in passed} >= {"telemetry_rate_hz", "lidar_rate_hz", "yaw_offset_deg"}
    for param, var in passed:
        assert re.search(rf'^{var}="\$\(ros_double "\${var}"\)"', script, re.M), (
            f"{param} is passed from ${var}, which is not converted with ros_double"
        )


def _ros_double(value):
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("bash not available")
    func = re.search(r"^ros_double\(\) \{.*?^\}", _read(SCRIPT), re.S | re.M).group(0)
    return subprocess.run(
        [bash, "-c", func + '\nros_double "$1"', "_", value], capture_output=True, text=True
    )


@pytest.mark.parametrize("value", ["0", "90", "180", "270", "-90", "1", "2", "1.0", "0.5", "+3", "270.0"])
def test_numbers_become_ros_doubles(value):
    result = _ros_double(value)
    assert result.returncode == 0, result.stderr
    out = result.stdout.strip()
    assert ROS_DOUBLE.match(out), out
    assert float(out) == float(value)
    yaml = pytest.importorskip("yaml")
    assert isinstance(yaml.safe_load(out), float)


@pytest.mark.parametrize("value", ["", "abc", "270deg", "1e3", "2.", ".5", "nan"])
def test_non_numbers_are_refused(value):
    assert _ros_double(value).returncode != 0
