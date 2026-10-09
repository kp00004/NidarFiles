"""Find the RPLIDAR among the Jetson's USB serial ports by asking each one
for its device info (RPLIDAR GET_INFO). The RPLIDAR's USB adapter and the
MicroLR900 radio are both CP2102 chips; their /dev/serial/by-id names differ
on our units but CP2102 adapters MAY share a name, so this probe does not
depend on device names at all.

    python3 -m nidar_autonomy.rplidar_probe
prints shell assignments for start_jetson.sh, e.g.
    LIDAR_PORT=/dev/ttyUSB1
    LIDAR_BAUD=115200
    LIDAR_INFO='model 0x28 fw 1.29 hw 7 serial ...'
    OTHER_PORTS='/dev/ttyUSB0'
(LIDAR_PORT empty if no RPLIDAR answered). The probe writes two short
RPLIDAR commands to each port; on the radio port they go out as a few
stray bytes that the GCS ignores.
"""

from __future__ import annotations

import glob
import os
import time
from typing import List, Optional, Tuple

from .rplidar_protocol import CMD_GET_INFO, CMD_STOP, DESCRIPTOR_LEN, INFO_LEN, DeviceInfo, command, parse_info

BAUDS = (115200, 256000)  # A2M8 / A1: 115200; A2M12: 256000


def candidate_ports() -> List[str]:
    ports = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    return [os.path.realpath(p) for p in ports]


def probe(port: str, baud: int, timeout_s: float = 0.6) -> Optional[DeviceInfo]:
    import serial  # python3-serial

    try:
        with serial.Serial(port, baud, timeout=0.05) as ser:
            ser.write(command(CMD_STOP))
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write(command(CMD_GET_INFO))
            data = b""
            deadline = time.monotonic() + timeout_s
            while len(data) < DESCRIPTOR_LEN + INFO_LEN and time.monotonic() < deadline:
                data += ser.read(DESCRIPTOR_LEN + INFO_LEN - len(data))
            start = data.find(b"\xa5\x5a")
            return parse_info(data[start:]) if start >= 0 else None
    except (OSError, serial.SerialException):
        return None


def find_lidar(ports: List[str]) -> Optional[Tuple[str, int, DeviceInfo]]:
    for port in ports:
        for baud in BAUDS:
            info = probe(port, baud)
            if info is not None:
                return port, baud, info
    return None


def main() -> None:
    ports = candidate_ports()
    found = find_lidar(ports)
    if found is None:
        print("LIDAR_PORT=")
        print("LIDAR_BAUD=")
        print("LIDAR_INFO=''")
        print(f"OTHER_PORTS='{' '.join(ports)}'")
        return
    port, baud, info = found
    print(f"LIDAR_PORT={port}")
    print(f"LIDAR_BAUD={baud}")
    print(f"LIDAR_INFO='model 0x{info.model:02X} fw {info.firmware} hw {info.hardware} serial {info.serial}'")
    print(f"OTHER_PORTS='{' '.join(p for p in ports if p != port)}'")


if __name__ == "__main__":
    main()
