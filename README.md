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
- 🐳 **Dockerized Simulation** - Run a full, end-to-end simulation with Gazebo using a single Docker command.

## 🚀 Quick Start

> **Note**
> The following instructions are for deploying the software on the physical Raspberry Pi robot. For a simulation-based workflow, please see the **[End-to-End Simulation](#-end-to-end-simulation)** section below.

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

## 🐳 End-to-End Simulation

You can run the entire system (robot backend + dashboard) on your local machine using our Dockerized Gazebo simulation. This is the recommended way to test new features without needing the physical hardware.

### Prerequisites

-   [Docker](https://docs.docker.com/get-docker/) installed and running.
-   An X11 server running on your host machine.
    -   **Linux:** This is usually running by default.
    -   **Windows:** Use [WSLg](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps) (recommended) or install a server like [VcXsrv](https://sourceforge.net/projects/vcxsrv/).
    -   **macOS:** Install and run [XQuartz](https://www.xquartz.org/).

### Running the Simulation

You will need to run the ROS 2 simulation and the web dashboard in two separate terminals.

#### **Terminal 1: Launch the ROS 2 Simulation**

This command builds the Docker image (if it's the first time) and starts the Gazebo simulator and all the ROS 2 nodes.

1.  **Navigate to the Robot App Directory:**
    ```bash
    cd apps/robot
    ```

2.  **Allow Local X11 Connections:**
    This step is required to allow the Gazebo GUI from the container to be displayed on your host screen.
    ```bash
    xhost +local:
    ```

3.  **Run with Docker Compose:**
    ```bash
    docker compose up --build
    ```
    -   After a few moments, you should see the Gazebo application window open with the robot in a simple, empty world.
    -   You can close the simulation by pressing `Ctrl+C`.

#### **Terminal 2: Launch the Web Dashboard**

1.  **Navigate to the Dashboard Directory:**
    ```bash
    cd apps/dashboard
    ```
2.  **Install Dependencies:**
    ```bash
    npm install
    ```
3.  **Start the Development Server:**
    ```bash
    npm run dev
    ```

### Connecting the Dashboard to the Simulation

1.  Open your web browser and navigate to the URL provided by the `npm run dev` command (usually `http://localhost:5173`).
2.  Since both the simulation and the dashboard are running on your local machine, use the default `localhost` connection settings:
    -   **ROS Bridge URL:** `ws://localhost:9090`
    -   **Camera Stream URL:** `http://localhost:8080/stream?topic=/camera/image_raw`
3.  Click **Connect**.
4.  You should now see the live camera feed from the Gazebo world and be able to drive the simulated robot with the virtual joystick!

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is open source and available under the MIT License.
