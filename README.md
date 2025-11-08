# 🤖 ROS 2 Robot Dashboard

A real-time web-based control dashboard for Raspberry Pi robots running ROS 2. Control your robot remotely through an intuitive browser interface with live camera feed, virtual joystick, and system telemetry.

![Dashboard Preview](https://img.shields.io/badge/ROS%202-Humble-blue?style=for-the-badge&logo=ros&logoColor=white)
![React](https://img.shields.io/badge/React-18.3-61DAFB?style=for-the-badge&logo=react&logoColor=black)
![TypeScript](https://img.shields.io/badge/TypeScript-5.5-3178C6?style=for-the-badge&logo=typescript&logoColor=white)

## 🌟 Features

- 🕹️ **Virtual Joystick Control** - Smooth, responsive touch/mouse control for robot movement
- 📹 **Live Camera Feed** - Real-time video streaming from robot's camera
- 📊 **System Telemetry** - Monitor CPU temperature, usage, memory, and uptime
- 🔌 **Easy Connection** - Simple WebSocket connection to ROS Bridge
- 📱 **Responsive Design** - Works on desktop, tablet, and mobile devices
- 🎨 **Modern UI** - Beautiful dark theme with real-time status indicators

## 🚀 Quick Start

### Step 1: Robot Setup (Raspberry Pi)

**1. Create a ROS 2 Workspace and Link the Package:**

First, create a `ros2_ws` directory and a `src` folder inside it. Then, symlink the `pi_robot` package from this project into the `src` folder.

```bash
mkdir -p ~/ros2_ws/src
ln -s "$(pwd)/apps/robot/src/pi_robot" ~/ros2_ws/src/pi_robot
```

**2. Build the ROS 2 Package:**

Navigate to your workspace, build the `pi_robot` package, and source the setup file.

```bash
cd ~/ros2_ws
colcon build --packages-select pi_robot --symlink-install
source install/setup.bash
```

**3. Enable pigpio daemon (required for motor control):**

```bash
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

**3. Launch the robot nodes:**

This single command starts all the required ROS 2 nodes, including the ROS Bridge and the Web Video Server.

```bash
ros2 launch pi_robot robot_bringup.launch.py
```

### Step 2: Dashboard Setup (Development Machine)

**1. Navigate to the Dashboard Directory:**

All commands for the dashboard should be run from the `apps/dashboard` directory.

```bash
cd apps/dashboard
```

**2. Install Dependencies:**

```bash
npm install
```

**3. Start the Development Server:**

```bash
npm run dev
```

**4. Open the Dashboard:**

- Navigate to `http://localhost:5173` in your browser.

**5. Connect to Your Robot:**

- In the connection panel, enter your Raspberry Pi's IP address.
  - **Hint:** You can find your Pi's IP address by running `hostname -I` on the Raspberry Pi.
- **ROS Bridge URL:** `ws://<RASPBERRY_PI_IP>:9090`
- **Camera Stream URL:** `http://<RASPBERRY_PI_IP>:8080/stream?topic=/camera/image_raw`
- Click **Connect**.
- You should now see the camera feed and be able to control the robot!

## 📚 Documentation

For more detailed information, please refer to the following documents:

-   [**System Architecture**](./docs/Architecture.md)
-   [**Synergy and Data Flow**](./docs/Synergy.md)
-   [**ROS 2 Package (`pi_robot`)**](./apps/robot/src/pi_robot/README.md)
-   [**Web Dashboard**](./apps/dashboard/README.md)

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is open source and available under the MIT License.
