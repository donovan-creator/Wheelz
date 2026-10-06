# Wheelz — ROS 2

Wheelz is a small differential-drive robot with an ESP8266 controlling the motors,
encoders and MPU6050, and an ESP32-CAM providing video. The host software now runs
as ROS 2 nodes in Ubuntu through WSL2. **Neither ESP firmware file was changed.**

Start with the [ROS setup and operation guide](ros2_ws/README.md). For the Toolbox Automatic tab, use the [app connection guide](ros2_ws/APP_SETUP.md).

```text
ROS keyboard / opt-in PPO policy -> /cmd_vel -> wheelz_driver -> ESP8266 HTTP
ESP8266 /counts + /imu -> wheelz_driver -> encoder counts, IMU, calibrated odometry
ESP32-CAM MJPEG -> wheelz_camera -> ROS images
ROS topics -> ros2 bag record
```

- [ros2_ws/](ros2_ws/): active host runtime, launch/configuration, ESP emulator and tests.
- [Firmware/](Firmware/): existing Arduino firmware, retained unchanged.
- [PC/reinforcement_learning/](PC/reinforcement_learning/): five-action PPO training,
  evaluation and export. A trained goal-navigation model is included; see the
  [model report](ros2_ws/src/wheelz_ros/models/MODEL_CARD.md).
- [PC/](PC/): previous receiver/cloud relay and detection placeholder, retained for reference.
  These are not required by the ROS launch.
- [3d print/](3d%20print/): chassis and fabrication assets.

The existing firmware supports five full-power actions, not proportional velocity
control. The bridge maps ROS commands to those actions. It cannot provide
closed-loop wheel speed, seamless curved paths, or a hardware disconnect watchdog.
A lower `cmd_vel` value does **not** reduce motor power.
