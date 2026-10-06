# Wheelz ROS 2 host setup

This workspace replaces the phone relay, Python WebSocket viewer and cloud
control/logging path with ROS 2 topics, services, camera nodes and rosbag.
Both ESP sketches are unchanged. Run all ROS processes in the same WSL Ubuntu
distribution. The boards continue speaking their original HTTP/MJPEG protocols;
they do not need ROS or micro-ROS installed.

## Installed on this PC

The WSL distribution named `Ubuntu` runs Ubuntu 26.04. ROS 2 **Lyrical** and the
build/runtime dependencies are installed there. The package was built and tested
in that distribution. The installer also supports **Jazzy on Ubuntu 24.04**;
Jazzy compatibility is intended, but the local integration test used Lyrical.

From PowerShell, open Ubuntu:

```powershell
wsl -d Ubuntu
```

Then, inside Ubuntu:

```bash
cd /mnt/c/Users/donov/Desktop/stuff/career/Projects/Wheelz/ros2_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch wheelz_ros bringup.launch.py
```

The default launch runs **only a loopback ESP emulator** and a synthetic camera.
It publishes real ROS topics, with artificial encoder calibration for the demo.
It does not contact physical hardware. The emulator is an interface test, not a
physics simulator or an RL training environment.

A PowerShell shortcut that handles sourcing for you:

```powershell
wsl -d Ubuntu -- bash /mnt/c/Users/donov/Desktop/stuff/career/Projects/Wheelz/ros2_ws/scripts/run.sh
```

On another machine, install dependencies and build inside Ubuntu:

```bash
bash scripts/install_ros.sh
bash scripts/build.sh
source install/setup.bash
```

The installer adds the official ROS apt source and installs packages; it does
not modify your firmware, network configuration, shell startup files or Linux
distribution. Run `bash scripts/build.sh` after adding nodes or changing packaging.

## Toolbox app

The app now has Manual and Automatic coordinate-entry tabs. Follow
[the app setup guide](APP_SETUP.md) to launch the paired ROS host and connect
your phone. Use the app launch instead of starting a second policy node.

## Keyboard control

In a second Ubuntu terminal, source the two setup files above, then:

```bash
ros2 service call /wheelz_driver/enable std_srvs/srv/SetBool "{data: true}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use `i` for forward, `,` for backward, `j/l` for turns, and `k` to stop.
Hold a movement key to generate repeated keypresses. The installed keyboard node
publishes on keypress, so releasing a key lets the bridge's 0.5-second command
timeout stop the robot; press k for an immediate stop request. Initial keyboard
repeat delay can cause a brief pause. Speed-adjustment keys cannot change the
firmware's fixed motor power.
The keyboard node publishes `geometry_msgs/msg/Twist`, the bridge's input type.
Keep only one command publisher active. ROS does not arbitrate competing teleop
and autonomy publishers in this package.

To stop **and disable** further motion commands:

```bash
ros2 service call /wheelz_driver/stop std_srvs/srv/Trigger "{}"
```

This service confirms a stop request was queued, not that the robot physically
stopped. Check `/diagnostics` for connection state and HTTP acknowledgment.
Re-enable explicitly to accept movement again. The driver starts disabled, clears
old commands on enable, and disables itself after a communication/sensor failure.
Reconnection alone never enables movement.

## Connect the existing robot

1. Put the PC and boards on a network where WSL can reach their IP addresses.
   The ESP8266 may join its configured hotspot or create its fallback AP.
2. Stop the demo launch. Confirm the ESP8266 responds to
   `curl --max-time 2 http://ESP8266_IP/ping` from Ubuntu.
3. Use the **actual working MJPEG URL** from the deployed ESP32-CAM. A common
   example is `http://ESP32_IP:81/stream`, but this repo's ESP32 sketch lacks its
   camera pin configuration and server implementation, so that URL cannot be
   inferred or guaranteed from the checked-in sketch.
4. Launch with hardware mode selected explicitly:

```bash
ros2 launch wheelz_ros bringup.launch.py \
  mock:=false robot_url:=http://ESP8266_IP \
  camera_url:=http://ESP32_IP:81/stream
```

Omit `camera_url` if you only want motors/sensors. Replace the address placeholders
with actual addresses. No physical addresses are preconfigured. Do not run the
old app/backend as a second motor controller while using ROS.

Initially leave the wheels off the ground to check forward direction, yaw and
encoder polarity. This bridge sends the firmware's existing **full-power**
commands. A `linear.x` of 0.05 and 0.5 both request the same `/forward` action.
Angular commands take priority over linear commands; combined inputs turn in
place rather than trace a curve. Unsupported axes and nonfinite values request
stop. Deadbands are configured in `config/robot.yaml`.

The checked-in `/left` drives left forward/right backward, and `/right` does the
opposite. Consequently positive ROS yaw maps to `/right` by default. Verify the
assembled robot; set `positive_yaw_action: left` if necessary. The firmware's
turn handlers send **no HTTP reply**. The bridge accepts a sent but unacknowledged
turn, exposes `acknowledged=False` in diagnostics, and checks sensor connectivity;
it cannot prove that the turn executed.

### Find the ESP8266 address

The sketch contains Wi-Fi credentials, not a fixed station IP. The hotspot/router
assigns the address. Look in its connected-device list, or open Arduino Serial
Monitor at **115200 baud** and reset the board: the sketch prints
`WiFi.localIP()` after connecting. Its fallback network is named `RobotCar`;
while connected to that network, try `http://192.168.4.1/ping` (the
[ESP8266 default AP address](https://arduino-esp8266.readthedocs.io/en/3.1.0/esp8266wifi/soft-access-point-class.html)).
The address can change when joining a different hotspot; update `robot_url`.

The existing sketch shares RX/TX pins with the right encoder and also uses
Serial. If Serial output or right-encoder readings are unreliable, inspect that
existing pin-sharing arrangement during hardware calibration. This migration
does not change the board's pins or firmware.

### Stop behavior and limitations

After 0.5 seconds without a fresh command, the host requests `/stop`. It also
requests stop at startup, disable, detected connection failure and clean shutdown.
HTTP requests are serialized on a worker thread with bounded socket waits, so a
stop can be delayed by an in-flight request or host scheduling.

**There is no firmware watchdog.** If Wi-Fi fails, the host crashes or WSL is
terminated, the board can continue its last motor action indefinitely. Have a
physical power cutoff available. Host timeout logic cannot fix this while the
firmware remains unchanged. Do not use this bridge for unattended driving.

## Topics and services

| Interface | ROS type | Meaning |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Discrete action selection from linear.x and angular.z |
| `/wheelz/encoder_counts` | `std_msgs/msg/Int64MultiArray` | Raw signed counts, [left, right] |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | m/s² and rad/s, in IMU sensor axes |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry, only with configured calibration |
| `/tf` | `tf2_msgs/msg/TFMessage` | odom -> base_link, when odometry is enabled |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Decoded BGR8 video |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Original JPEG frames |
| `/wheelz/action` | `std_msgs/msg/String` | Last action sent, not measured motion |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Connection, enable state, acknowledgment/errors |
| `/wheelz_driver/enable` | `std_srvs/srv/SetBool` | Enable/disable command acceptance |
| `/wheelz_driver/stop` | `std_srvs/srv/Trigger` | Disable and queue stop |

IMU and image topics use sensor-data QoS (best effort). The camera reconnects
after stream loss and keeps only the latest frame. Encoder and IMU timestamps
are host receipt times; the firmware has no sample timestamps. Raw encoder
arrays have no timestamp field; recorded bag receive times can be used.

The IMU's orientation covariance begins with -1 (orientation unavailable).
Other IMU covariances are unknown. Conversion assumes the MPU6050 library's
usual +/-2g and +/-250 deg/s defaults; change the scale parameters if your
deployed library uses different ranges. No IMU fusion is performed.

## Calibrate odometry

Real-hardware odometry is deliberately disabled by
`ticks_per_revolution: 0.0`. Do not copy the demo's 360 value to the robot.

Copy the configuration and edit your local calibration:

```bash
cp src/wheelz_ros/config/robot.yaml config.local.yaml
```

Measure counts over several complete output-wheel revolutions using
`ros2 topic echo /wheelz/encoder_counts`, then divide by the number of revolutions.
Use the count convention in the existing firmware: rising edges of encoder A,
not four-edge quadrature. Set each encoder sign so forward wheel motion produces
positive distance. Measure wheel diameter and tire-center track width; the
43 mm diameter and 166.32 mm track defaults come from existing project data and
the track is only an estimate.

```bash
ros2 launch wheelz_ros bringup.launch.py mock:=false \
  robot_url:=http://ESP8266_IP config:="$PWD/config.local.yaml"
```

Parameters are read-only at runtime; restart after edits. Odometry handles
signed counter wrap and rebaselines after a link failure or an implausible count
jump. Small counter resets cannot always be distinguished from motion with this
protocol. Restart the node after a known board reboot. The pose starts at zero
and is not a globally localized position. Covariances are conservative placeholders,
not measured noise statistics.

Only the odom -> base_link transform is supplied. IMU mounting transforms,
camera mounting transforms and camera intrinsics require physical measurements;
none are fabricated. Add these before RViz overlays, sensor fusion or SLAM.
There is no `CameraInfo` publisher until a calibration is supplied.

## Inspect and record

```bash
ros2 topic list
ros2 topic echo /diagnostics
ros2 topic echo /wheelz/encoder_counts
ros2 topic echo /imu/data_raw --qos-reliability best_effort
ros2 topic hz /camera/image_raw --qos-reliability best_effort
mkdir -p bags
ros2 bag record -o bags/wheelz_run /cmd_vel /wheelz/action \
  /wheelz/encoder_counts /imu/data_raw /odom /tf /diagnostics \
  /camera/image_raw/compressed
```

Use a new output directory for each recording. Bag recording replaces the old
cloud CSV logger and keeps typed sensor messages with timestamps. Avoid replaying
recorded `/cmd_vel` into an enabled physical bridge. Inspect bags in a separate
ROS domain or with the hardware driver stopped.

Optional video viewer: install `ros-lyrical-rqt-image-view`, run
`ros2 run rqt_image_view rqt_image_view`, and select `/camera/image_raw`.
Use Jazzy's package name if running Ubuntu 24.04.

## Learned goal policy

A trained five-action PPO model is included in `src/wheelz_ros/models/`.
It reached and stopped at 200/200 held-out simulated goals; see the
[model card](src/wheelz_ros/models/MODEL_CARD.md) for metrics and limitations.
Training and held-out evaluation run on the existing Windows Python environment;
ROS inference runs in WSL using NumPy, without PyTorch or ONNX dependencies.

The policy is opt-in and is **not started by bringup**. It uses calibrated
`/odom` and `geometry_msgs/msg/PoseStamped` goals on `/goal_pose`, in the `odom`
frame. There is no map or obstacle avoidance. Its default output topic is
`/wheelz/policy_cmd_vel`, so you can inspect proposals without sending them to the
motor bridge. Goals more than 4 m from the current pose are rejected.

After sourcing the ROS workspace and starting a driver with odometry:

```bash
ros2 run wheelz_ros policy
```

In another sourced Ubuntu terminal:

```bash
ros2 service call /wheelz_policy/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
ros2 topic echo /wheelz/policy_cmd_vel
```

A new goal is required after enabling. The policy stops and disables on stale or
invalid odometry, an invalid goal, a 40-second navigation timeout, or goal arrival.
It also has an independent near-goal stop guard. The model's simulated success
numbers were measured without that guard. An odometry failure does not resume
automatically. Stop the policy explicitly with:

```bash
ros2 service call /wheelz_policy/enable std_srvs/srv/SetBool "{data: false}"
```

For a supervised test **after calibration**, stop the preview policy node and
launch it with `--ros-args -r wheelz/policy_cmd_vel:=cmd_vel`. Enable the bridge and
policy separately, then send a goal. Do not run keyboard control concurrently.
The node never calls the bridge's enable service. Simulation success does not
make the uncalibrated physical robot safe or accurate; the existing firmware
still has no disconnect watchdog.

To watch the learned model in its simulator, use the existing Windows RL venv:

```powershell
cd C:\Users\donov\Desktop\stuff\career\Projects\Wheelz\pc\reinforcement_learning
.\.venv\Scripts\python.exe evaluate.py --episodes 5 --watch
```

The [RL guide](../PC/reinforcement_learning/README.md) explains training, resuming,
exporting and the exact five-action contract. The earlier continuous SAC code
and models were preserved. The first discrete experiment exposed a reward loop;
the delivered model was trained after removing that incentive and selecting
checkpoints by completed goals.

A recorded robot dataset is not required for this simulator-based training.
ROS bags remain useful for measuring hardware behavior and checking simulated
assumptions. The previous detector was a placeholder; a future detector can
subscribe to `/camera/image_raw`.

## Tests

```bash
bash scripts/test.sh
```

This runs colcon with pytest explicitly (needed with newer setuptools), tests
HTTP parsing, SI units, turn handlers without replies, encoder wrap/polarity,
odometry rebaselining, command timeout, disabled state, recovery and shutdown.
An integration test starts the real ROS launch against loopback ESPs, subscribes
through DDS, verifies images/IMU/odometry, sends motion commands and exercises
enable/stop services. Policy tests also exercise goal validation and stale-odometry cancellation. The tests never enable physical hardware.
Port 8080 must be free for the launch test; stop the demo first.

For core-only tests without ROS, from `src/wheelz_ros`:

```bash
python3 -m unittest discover -s test -v
```

The ROS test is skipped when `rclpy` is unavailable.

## WSL networking

Normal WSL NAT is enough when every ROS node runs in WSL and both ESP IPs are
reachable by outbound HTTP. Keep keyboard control, bag recording and viewers
inside the same distro. A VPN, firewall or Wi-Fi client isolation can block board
access; diagnose with `curl` before changing ROS settings. For ROS nodes on other
machines, Windows/WSL multicast may need additional configuration; mirrored
networking is an optional Windows feature, not enabled by these scripts.

References: [ROS installation](https://docs.ros.org/en/rolling/Get-Started/Installation/Ubuntu-Install-Debs.html),
[Lyrical release](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021),
[WSL networking](https://learn.microsoft.com/en-us/windows/wsl/networking).
