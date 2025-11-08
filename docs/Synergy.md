# Synergy and Data Flow

This document explains how the different components of the ROS 2 Robot Dashboard work together to provide real-time control and monitoring. It details the data flow for the key functionalities of the system.

## Data Flow

### 1. Movement Control Flow

The movement control is initiated from the web dashboard and executed by the robot's motors. The data flow is as follows:

```
User Interaction → Virtual Joystick (nipplejs)
    ↓
JavaScript Event {x, y} → Convert to linear/angular velocity
    ↓
ROSLIB.Message → WebSocket (Port 9090)
    ↓
ROS Bridge → /cmd_vel Topic (geometry_msgs/Twist)
    ↓
diff_drive Node → Calculate left/right wheel speeds
    ↓
pigpio GPIO → PWM signals to L298N
    ↓
DC Motors → Robot Movement
```

### 2. Video Streaming Flow

The video stream is captured by the robot's camera and displayed on the web dashboard. The data flow is as follows:

```
IMX219 Camera Module → Capture frames
    ↓
camera_node (OpenCV) → Process frames
    ↓
Publish to /camera/image_raw (20 Hz)
    ↓
web_video_server → Convert to MJPEG stream
    ↓
HTTP Server (Port 8080) → Stream endpoint
    ↓
Browser <img> tag → Display live feed
```

### 3. Telemetry Flow

The telemetry data is collected from the Raspberry Pi and displayed on the web dashboard. The data flow is as follows:

```
Raspberry Pi System → CPU temp, usage, memory
    ↓
telemetry_node (psutil) → Read system stats
    ↓
JSON.stringify → /telemetry Topic (1 Hz)
    ↓
ROS Bridge → WebSocket to browser
    ↓
React State Update → Display in TelemetryPanel
```
