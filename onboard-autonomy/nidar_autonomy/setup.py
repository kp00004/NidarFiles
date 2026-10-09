from setuptools import find_packages, setup

package_name = "nidar_autonomy"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TeamArdra",
    maintainer_email="raj.tibarewala@gmail.com",
    description=(
        "Onboard autonomy nodes for NIDAR AirMouse. Phase 0: command "
        "handling, mission state reporting, and heartbeat only."
    ),
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "command_node = nidar_autonomy.command_node:main",
            "mission_state_node = nidar_autonomy.mission_state_node:main",
            "heartbeat_node = nidar_autonomy.heartbeat_node:main",
            "checkpoint2_arm_test = nidar_autonomy.checkpoint2_arm_test:main",
            "telemetry_bridge_node = nidar_autonomy.telemetry_bridge_node:main",
            "coverage_tracker_node = nidar_autonomy.coverage_tracker_node:main",
            "frontier_explorer_node = nidar_autonomy.frontier_explorer_node:main",
            "geofence_monitor_node = nidar_autonomy.geofence_monitor_node:main",
            "simulation_node = nidar_autonomy.simulation_node:main",
            "radio_command_node = nidar_autonomy.radio_command_node:main",
            "lidar_node = nidar_autonomy.lidar_node:main",
        ],
    },
)
