# `pi_robot` ROS 2 Package

This package contains the ROS 2 nodes to control the Raspberry Pi robot and stream data to the web-based dashboard. It is the core of the robot's backend functionality. For a complete overview of the project, see the [main `README.md`](../../../README.md).

## Nodes

This package includes the following nodes, which are all started by the main launch file:

-   **`diff_drive_node`**: Controls the robot's movement based on `/cmd_vel` commands.
-   **`camera_node`**: Captures video from the camera and publishes it to the `/camera/image_raw` topic.
-   **`telemetry_node`**: Gathers and publishes system telemetry data like CPU usage and temperature.

The main launch file also starts `rosbridge_server` and `web_video_server` for communication with the dashboard.

## Launching

There are two primary ways to launch the nodes in this package:

### 1. Hardware Bringup (on Raspberry Pi)

This command starts all the nodes required to run the physical robot.

```bash
ros2 launch pi_robot robot_bringup.launch.py
```

### 2. Simulation Bringup (with Gazebo)

This command starts the Gazebo simulator and the necessary nodes for end-to-end testing without hardware. For a complete guide, see the [End-to-End Simulation](../../../../README.md#️-end-to-end-simulation) section in the main `README`.

```bash
ros2 launch pi_robot robot_simulation.launch.py
```

## Configurable Parameters

You can modify the robot's parameters when launching the `robot_bringup.launch.py` file.

### Differential Drive

-   `wheel_radius`: The radius of the robot's wheels (in meters). Default: `0.033`
-   `wheel_base`: The distance between the robot's wheels (in meters). Default: `0.16`

Example:
```bash
ros2 launch pi_robot robot_bringup.launch.py wheel_radius:=0.040 wheel_base:=0.18
```

### Camera

-   `frame_rate`: The frame rate of the camera. Default: `20.0`
-   `width`: The width of the camera image. Default: `640`
-   `height`: The height of the camera image. Default: `480`

Example:
```bash
ros2 launch pi_robot robot_bringup.launch.py frame_rate:=30.0 width:=1280 height:=720
```
