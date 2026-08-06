# JetCobot official URDF coordinate diagnostics

This package is isolated from the existing `arm` and pick/place packages. It
does not command robot motion. One process owns the serial port and captures:

- measured `get_angles()`;
- measured `get_coords()`;
- official URDF `base_link -> 6_Link`;
- the candidate fixed transform `6_Link -> controller_coords`.

The external URDF is loaded with an `official/` frame prefix, so it does not
collide with the existing TF tree.

## Safety

Stop every other process that uses the robot serial port before launching.
The capture service rejects a sample while `is_moving()` reports motion.
Move the robot only with an independently verified safe procedure, wait for a
complete stop, and then capture.

## Build

```bash
cd ~/poter_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select jetcobot_model_diagnostics
source install/setup.bash
```

## Launch

```bash
ros2 launch jetcobot_model_diagnostics \
  official_urdf_coords_test.launch.py \
  serial_port:=/dev/ttyUSB1
```

The default URDF is:

```text
~/bizlink-Yahboom.jetcobot_ws/src/jetcobot_description/urdf/jetcobot.urdf
```

Override it with `official_urdf_path:=/absolute/path/to/robot.urdf`.

## Capture

At each completely stopped, user-verified safe robot pose:

```bash
ros2 service call /official_urdf_test/capture \
  std_srvs/srv/Trigger '{}'
```

Collect at least 12 samples distributed across J1 near -90, -45, and 0
degrees. Vary J2-J6 as safely possible.

## Analyze

```bash
ros2 service call /official_urdf_test/analyze \
  std_srvs/srv/Trigger '{}'
```

Results are written under `~/poter_ws/test_results/` as CSV and JSON.

## Clear with backup

```bash
ros2 service call /official_urdf_test/clear \
  std_srvs/srv/Trigger '{}'
```

The existing CSV is renamed to a timestamped backup rather than deleted.
