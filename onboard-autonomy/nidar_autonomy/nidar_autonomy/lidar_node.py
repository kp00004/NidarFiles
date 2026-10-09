"""RPLIDAR A2 driver node: reads the LiDAR over USB serial (standard SCAN
mode, rplidar_protocol.py) and publishes every 360° turn as a ROS
sensor_msgs/LaserScan on /scan (frame "laser", angle_min -pi, 1° bins,
counter-clockwise from the vehicle's nose, inf = no return).

Plain Python + pyserial, so nothing extra has to be installed on the
Jetson (rplidar_ros would also work: radio_command_node only reads /scan).

Parameters:
  serial_port     the RPLIDAR's port (start_jetson.sh finds it with
                  rplidar_probe -- the radio uses the same USB chip)
  baud            115200 (A2M8/A1) or 256000 (A2M12)
  yaw_offset_deg  direction of the LiDAR's 0° mark relative to the nose,
                  clockwise (0 = mounted facing forward)
  range_min_m / range_max_m   0.15 / 12.0 (A2M8)
  motor_pwm       660 (SLAMTEC default)
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .rplidar_protocol import (
    CMD_SCAN,
    CMD_STOP,
    DEFAULT_MOTOR_PWM,
    DESCRIPTOR_LEN,
    SCAN_TYPE,
    ScanParser,
    command,
    laserscan_bins,
    motor_pwm_command,
    parse_descriptor,
)
from .topics import SCAN_TOPIC

BINS = 360
_REOPEN_S = 2.0


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")


class LidarNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_node")
        self._port = self.declare_parameter("serial_port", "/dev/ttyUSB1").value
        self._baud = int(self.declare_parameter("baud", 115200).value)
        self._yaw_offset = float(self.declare_parameter("yaw_offset_deg", 0.0).value)
        self._range_min = float(self.declare_parameter("range_min_m", 0.15).value)
        self._range_max = float(self.declare_parameter("range_max_m", 12.0).value)
        self._pwm = int(self.declare_parameter("motor_pwm", DEFAULT_MOTOR_PWM).value)
        self._pub = self.create_publisher(LaserScan, SCAN_TOPIC, 5)
        self._stop = threading.Event()
        self._serial = None
        self._turns = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"[{_now()}] lidar_node: {self._port} @ {self._baud}, yaw offset {self._yaw_offset:g} deg -> {SCAN_TOPIC}"
        )

    def _report(self) -> None:
        self.get_logger().info(f"[{_now()}] lidar: {self._turns} scans published in the last 5 s")
        self._turns = 0

    # -- serial -----------------------------------------------------------------

    def _start(self):
        import serial

        ser = serial.Serial(self._port, self._baud, timeout=0.1)
        ser.write(command(CMD_STOP))
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.dtr = False  # A2 adapter: DTR low -> motor on
        ser.write(motor_pwm_command(self._pwm))
        time.sleep(1.0)  # let the motor spin up
        ser.reset_input_buffer()
        ser.write(command(CMD_SCAN))
        header = b""
        deadline = time.monotonic() + 2.0
        while len(header) < DESCRIPTOR_LEN and time.monotonic() < deadline:
            header += ser.read(DESCRIPTOR_LEN - len(header))
        desc = parse_descriptor(header)
        if desc is None or desc.data_type != SCAN_TYPE:
            ser.close()
            raise OSError(f"no SCAN answer from {self._port} (got {header.hex()}) -- wrong port or baud?")
        return ser

    def _stop_device(self) -> None:
        ser, self._serial = self._serial, None
        if ser is None:
            return
        try:
            ser.write(command(CMD_STOP))
            ser.write(motor_pwm_command(0))
            ser.dtr = True  # motor off
            ser.close()
        except Exception:  # noqa: BLE001 -- port may be gone already
            pass

    def _run(self) -> None:
        warned = False
        while not self._stop.is_set():
            try:
                self._serial = self._start()
            except Exception as exc:  # noqa: BLE001 -- unplugged / wrong port
                if not warned:
                    self.get_logger().error(f"[{_now()}] LiDAR not started: {exc}; retrying every {_REOPEN_S:g} s")
                    warned = True
                self._stop.wait(_REOPEN_S)
                continue
            warned = False
            self.get_logger().info(f"[{_now()}] LiDAR scanning on {self._port}")
            parser = ScanParser()
            try:
                while not self._stop.is_set():
                    data = self._serial.read(1024)
                    for turn in parser.feed(data):
                        self._publish(turn)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"[{_now()}] LiDAR read failed: {exc!r}; restarting")
            finally:
                self._stop_device()

    def _publish(self, turn) -> None:
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_increment = 2.0 * math.pi / BINS
        msg.angle_min = -math.pi
        msg.angle_max = math.pi - msg.angle_increment
        msg.scan_time = 0.1
        msg.range_min = self._range_min
        msg.range_max = self._range_max
        msg.ranges = [float(r) for r in laserscan_bins(turn, self._yaw_offset, BINS, self._range_min)]
        self._pub.publish(msg)
        self._turns += 1

    def destroy_node(self) -> bool:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._stop_device()
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
