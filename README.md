# 🤖 ROS 2 Robot Dashboard

A real-time web-based control dashboard for Raspberry Pi robots running ROS 2. Control your robot remotely through an intuitive browser interface with live camera feed, virtual joystick, and system telemetry.

## 🌟 Features

- **🕹️ Virtual Joystick Control** - Smooth, responsive touch/mouse control for robot movement
- **📹 Live Camera Feed** - Real-time video streaming from robot's camera
- **📊 System Telemetry** - Monitor CPU temperature, usage, memory, and uptime
- **🔌 Easy Connection** - Simple WebSocket connection to ROS Bridge
- **📱 Responsive Design** - Works on desktop, tablet, and mobile devices
- **🎨 Modern UI** - Beautiful dark theme with real-time status indicators

## 🏗️ Architecture

```mermaid
graph TB
    subgraph Browser["🌐 Web Browser"]
        UI[React Dashboard]
        Camera[Camera Feed]
        Joy[Virtual Joystick]
        Telem[Telemetry]
    end

    subgraph ROS["🔌 ROS Bridge"]
        Bridge[rosbridge_websocket:9090]
        VideoServer[web_video_server:8080]
    end

    subgraph ROS2["🤖 ROS 2 (Raspberry Pi)"]
        CmdVel[/cmd_vel Topic]
        ImageTopic[/camera/image_raw]
        TeleTopic[/telemetry]
    end

    subgraph Nodes["⚙️ ROS 2 Nodes"]
        DiffDrive[diff_drive - Motor Control]
        CamNode[camera_node - IMX219]
        TeleNode[telemetry_node - Monitoring]
    end

    Joy --> Bridge --> CmdVel --> DiffDrive
    CamNode --> ImageTopic --> VideoServer --> Camera
    TeleNode --> TeleTopic --> Bridge --> Telem
```

## 🚀 Quick Start

### Prerequisites

**On your Raspberry Pi:**
- ROS 2 (Humble or later)
- rosbridge_server
- web_video_server
- Python packages: `pigpio`, `psutil`, `cv_bridge`

**On your development machine:**
- Node.js & npm

### Robot Setup (Raspberry Pi)

1. **Build your ROS 2 workspace:**
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

2. **Launch the robot nodes:**
```bash
ros2 launch robot_bringup robot_bringup.launch.py
```

3. **Start ROS Bridge (in new terminal):**
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

4. **Start video server (in new terminal):**
```bash
ros2 run web_video_server web_video_server
```

### Dashboard Setup

1. **Clone and install:**
```bash
git clone <YOUR_GIT_URL>
cd <YOUR_PROJECT_NAME>
npm install
```

2. **Start development server:**
```bash
npm run dev
```

3. **Connect to your robot:**
   - Open the dashboard in your browser
   - Enter your Raspberry Pi's IP address:
     - ROS Bridge: `ws://<PI_IP>:9090`
     - Camera: `http://<PI_IP>:8080/stream?topic=/camera/image_raw`
   - Click "Connect"

## 🔧 Configuration

### Hardware Pins (L298N Motor Driver)

| Component | GPIO Pin |
|-----------|----------|
| Left Motor IN1 | GPIO 17 |
| Left Motor IN2 | GPIO 18 |
| Left Motor EN | GPIO 12 (PWM) |
| Right Motor IN1 | GPIO 22 |
| Right Motor IN2 | GPIO 23 |
| Right Motor EN | GPIO 13 (PWM) |

### ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Robot movement commands |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera video feed |
| `/telemetry` | `std_msgs/String` | System telemetry data (JSON) |

## 📡 Communication Flow

1. **Movement Control:**
   - User moves virtual joystick → WebSocket message → ROS Bridge → `/cmd_vel` topic → `diff_drive` node → Motor GPIO

2. **Video Streaming:**
   - Camera → `camera_node` → `/camera/image_raw` → `web_video_server` → HTTP stream → Browser

3. **Telemetry:**
   - `telemetry_node` → `/telemetry` → ROS Bridge → WebSocket → Dashboard

## 🛠️ Development

### Project Structure

```
src/
├── components/
│   ├── CameraFeed.tsx      # Live video display
│   ├── VirtualJoystick.tsx # Touch/mouse joystick
│   ├── TelemetryPanel.tsx  # System stats display
│   └── ConnectionPanel.tsx # Connection management
├── pages/
│   └── Index.tsx           # Main dashboard
└── index.css               # Design system tokens
```

### Tech Stack

- **Frontend:** React + TypeScript + Vite
- **Styling:** Tailwind CSS + shadcn/ui
- **ROS Integration:** roslib.js
- **Joystick:** nipplejs
- **State Management:** React Hooks

## 🎨 Design System

The dashboard uses a cohesive design system with semantic tokens:
- Dark theme optimized for robotics
- Cyan/blue accent colors
- Responsive grid layout
- Real-time status indicators

## 📝 Project Info

**URL**: https://lovable.dev/projects/f0be7543-b124-4377-ad5a-5a8d78771707

## How can I edit this code?

There are several ways of editing your application.

**Use Lovable**

Simply visit the [Lovable Project](https://lovable.dev/projects/f0be7543-b124-4377-ad5a-5a8d78771707) and start prompting.

Changes made via Lovable will be committed automatically to this repo.

**Use your preferred IDE**

If you want to work locally using your own IDE, you can clone this repo and push changes. Pushed changes will also be reflected in Lovable.

The only requirement is having Node.js & npm installed - [install with nvm](https://github.com/nvm-sh/nvm#installing-and-updating)

Follow these steps:

```sh
# Step 1: Clone the repository using the project's Git URL.
git clone <YOUR_GIT_URL>

# Step 2: Navigate to the project directory.
cd <YOUR_PROJECT_NAME>

# Step 3: Install the necessary dependencies.
npm i

# Step 4: Start the development server with auto-reloading and an instant preview.
npm run dev
```

**Edit a file directly in GitHub**

- Navigate to the desired file(s).
- Click the "Edit" button (pencil icon) at the top right of the file view.
- Make your changes and commit the changes.

**Use GitHub Codespaces**

- Navigate to the main page of your repository.
- Click on the "Code" button (green button) near the top right.
- Select the "Codespaces" tab.
- Click on "New codespace" to launch a new Codespace environment.
- Edit files directly within the Codespace and commit and push your changes once you're done.

## What technologies are used for this project?

This project is built with:

- Vite
- TypeScript
- React
- shadcn-ui
- Tailwind CSS

## How can I deploy this project?

Simply open [Lovable](https://lovable.dev/projects/f0be7543-b124-4377-ad5a-5a8d78771707) and click on Share -> Publish.

## Can I connect a custom domain to my Lovable project?

Yes, you can!

To connect a domain, navigate to Project > Settings > Domains and click Connect Domain.

Read more here: [Setting up a custom domain](https://docs.lovable.dev/features/custom-domain#custom-domain)
