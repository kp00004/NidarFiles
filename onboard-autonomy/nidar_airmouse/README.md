# nidar_airmouse

One ROS 2 message, `SurvivorDetection.msg`, matching
`../../custom-gcs/docs/DATA_MODELS.md` §5 exactly:

```
int32 survivor_id
float64 x
float64 y
float64 confidence
```

## Why this package exists

The real `custom-gcs` backend subscribes to `/vision/survivors` as type
`nidar_airmouse/SurvivorDetection` (see
`custom-gcs/gcs/backend/app/ros_client.py`). On the real Jetson, that
subscription currently fails:

```
Unable to import nidar_airmouse.msg from package nidar_airmouse.
Caused by: No module named 'nidar_airmouse'
```

because this package didn't exist yet, only a JSON-only stand-in inside
`custom-gcs/sim/`. Building and installing this package on the Jetson
fixes that error — but note it only fixes the *subscription*; something
still needs to actually **publish** `SurvivorDetection` messages on
`/vision/survivors` (that's the survivor-detection node, not yet
written — see `../nidar_autonomy/`).

## Build (on the Jetson, inside a ROS 2 Humble colcon workspace)

```sh
mkdir -p ~/ros2_ws/src
ln -s ~/onboard-autonomy/nidar_airmouse ~/ros2_ws/src/nidar_airmouse
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nidar_airmouse
source install/setup.bash
```

Verify it installed correctly:

```sh
ros2 interface show nidar_airmouse/msg/SurvivorDetection
```

**Built and installed successfully via `colcon build` on the Jetson** —
verified as part of `onboard-autonomy`'s regular build
(`colcon build --packages-select nidar_airmouse nidar_autonomy`, rc=0).
Nothing publishes `SurvivorDetection` messages yet (see above) — that
remains a real gap, just not a build/install one.
