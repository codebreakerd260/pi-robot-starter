# pi_robot ROS 2 Package

This package contains the ROS 2 nodes to control the Raspberry Pi robot and stream data to the web-based dashboard.

## Prerequisites

- ROS 2 Humble Hawksbill
- `ros-humble-rosbridge-server`
- `ros-humble-web-video-server`
- `python3-pigpio`
- `python3-psutil`
- `python3-opencv`
- `cv-bridge`

## Building the Package

1.  Navigate to your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws
    ```
2.  Build the package:
    ```bash
    colcon build --packages-select pi_robot --symlink-install
    ```
3.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Launching the Robot

To launch all the nodes required for the robot, run the following command:

```bash
ros2 launch pi_robot robot_bringup.launch.py
```

This will start:
- The differential drive controller
- The camera node for video streaming
- The telemetry node for system monitoring
- The ROS Bridge server for communication with the web dashboard
- The Web Video Server for streaming the camera feed

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
