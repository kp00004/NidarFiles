"""Small ROS-message <-> plain-dict converters shared by the migrated
mapping/exploration nodes.

Added by the NIDAR Autonomy Migration (see CHECKPOINT/CURRENT_STATE.md).
`frontier_detector.py`, `occupancy_grid_environment.py`, and
`coverage_grid.py` all deliberately consume/produce the plain
`nav_msgs/OccupancyGrid`-as-seen-by-roslibjs dict shape
(`custom-gcs/docs/DATA_MODELS.md` Section 4) rather than a real
`nav_msgs.msg.OccupancyGrid`, so they stay importable and unit-testable
without a ROS install. The rclpy node wrappers that feed them real ROS
messages need a small, tested bridge between the two -- that's this module.
This module itself DOES import `nav_msgs.msg` and therefore requires ROS,
same as any other node-level file in this package.
"""
from __future__ import annotations

from typing import Any

from nav_msgs.msg import OccupancyGrid


def occupancy_grid_msg_to_dict(msg: OccupancyGrid) -> dict[str, Any]:
    """Convert a real `nav_msgs/OccupancyGrid` message into the plain dict
    shape `frontier_detector.detect_frontiers`,
    `occupancy_grid_environment.OccupancyGridEnvironment`, and
    `coverage_grid.CoverageGrid` all consume."""
    info = msg.info
    return {
        "header": {
            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
            "frame_id": msg.header.frame_id,
        },
        "info": {
            "resolution": info.resolution,
            "width": info.width,
            "height": info.height,
            "origin": {
                "position": {
                    "x": info.origin.position.x,
                    "y": info.origin.position.y,
                    "z": info.origin.position.z,
                },
                "orientation": {
                    "x": info.origin.orientation.x,
                    "y": info.origin.orientation.y,
                    "z": info.origin.orientation.z,
                    "w": info.origin.orientation.w,
                },
            },
        },
        "data": list(msg.data),
    }


def dict_to_occupancy_grid_msg(grid: dict[str, Any], *, stamp=None) -> OccupancyGrid:
    """Convert a plain occupancy-grid dict (e.g. from
    `coverage_grid.CoverageGrid.to_occupancy_grid_data()`) into a real
    `nav_msgs/OccupancyGrid` message, ready to publish. `stamp`, if given,
    should be a `builtin_interfaces.msg.Time` (e.g. `node.get_clock().now().to_msg()`)
    -- left to the caller since this module has no `Node`/clock access."""
    msg = OccupancyGrid()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = grid["header"]["frame_id"]
    info = grid["info"]
    msg.info.resolution = float(info["resolution"])
    msg.info.width = int(info["width"])
    msg.info.height = int(info["height"])
    position = info["origin"]["position"]
    msg.info.origin.position.x = float(position["x"])
    msg.info.origin.position.y = float(position["y"])
    msg.info.origin.position.z = float(position.get("z", 0.0))
    orientation = info["origin"].get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
    msg.info.origin.orientation.x = float(orientation["x"])
    msg.info.origin.orientation.y = float(orientation["y"])
    msg.info.origin.orientation.z = float(orientation["z"])
    msg.info.origin.orientation.w = float(orientation["w"])
    msg.data = [int(v) for v in grid["data"]]
    return msg
