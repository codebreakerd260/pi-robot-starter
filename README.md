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

## 🏗️ System Architecture

```mermaid
```

## 📋 Prerequisites

### Raspberry Pi Requirements
- **OS:** Ubuntu 22.04 Server (64-bit) or Raspberry Pi OS
- **ROS 2:** Humble Hawksbill or later
- **Hardware:**
  - Raspberry Pi 4 (2GB+ RAM recommended)
  - IMX219 Camera Module
  - L298N Motor Driver
  - 2x DC Motors with wheels
  - Power supply (batteries/power bank)

### Required ROS 2 Packages
```bash
sudo apt install ros-humble-rosbridge-server
sudo apt install ros-humble-web-video-server
```

### Required Python Packages
```bash
pip3 install pigpio psutil opencv-python cv-bridge
```

### Development Machine Requirements
- Node.js (v18 or later)
- npm or yarn
- Modern web browser

## 🚀 Quick Start

### Step 1: Robot Setup (Raspberry Pi)

**1. Build your ROS 2 workspace:**
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**2. Enable pigpio daemon (required for motor control):**
```bash
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

**3. Launch the robot nodes:**
```bash
ros2 launch robot_bringup robot_bringup.launch.py
```

**4. Start ROS Bridge (new terminal):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**5. Start video server (new terminal):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run web_video_server web_video_server
```

### Step 2: Dashboard Setup

**1. Clone the repository:**
```bash
git clone <YOUR_GIT_URL>
cd <YOUR_PROJECT_NAME>
```

**2. Install dependencies:**
```bash
npm install
```

**3. Start development server:**
```bash
npm run dev
```

**4. Open dashboard:**
- Navigate to `http://localhost:5173` in your browser

**5. Connect to your robot:**
- Enter your Raspberry Pi's IP address in the connection panel:
  - **ROS Bridge URL:** `ws://<RASPBERRY_PI_IP>:9090`
  - **Camera Stream URL:** `http://<RASPBERRY_PI_IP>:8080/stream?topic=/camera/image_raw`
- Click **Connect**
- You should see the camera feed and be able to control the robot!

## ⚙️ Configuration

### Hardware Wiring (L298N Motor Driver)

| Component | GPIO Pin | Function |
|-----------|----------|----------|
| Left Motor IN1 | GPIO 17 | Direction control |
| Left Motor IN2 | GPIO 18 | Direction control |
| Left Motor EN | GPIO 12 | PWM speed control |
| Right Motor IN1 | GPIO 22 | Direction control |
| Right Motor IN2 | GPIO 23 | Direction control |
| Right Motor EN | GPIO 13 | PWM speed control |

**Wiring Diagram:**
```
L298N          Raspberry Pi 4
------         --------------
IN1    <----   GPIO 17 (Left Motor)
IN2    <----   GPIO 18 (Left Motor)
ENA    <----   GPIO 12 (Left Motor PWM)
IN3    <----   GPIO 22 (Right Motor)
IN4    <----   GPIO 23 (Right Motor)
ENB    <----   GPIO 13 (Right Motor PWM)
GND    <----   GND
5V     <----   5V (or separate power)
```

### ROS 2 Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Variable | Robot velocity commands (linear.x, angular.z) |
| `/camera/image_raw` | `sensor_msgs/Image` | 20 Hz | Raw camera frames from IMX219 |
| `/telemetry` | `std_msgs/String` | 1 Hz | JSON: CPU temp, CPU%, memory%, uptime |

### Network Ports

| Service | Port | Protocol | Description |
|---------|------|----------|-------------|
| ROS Bridge | 9090 | WebSocket | Bidirectional ROS topic communication |
| Video Server | 8080 | HTTP | MJPEG video stream |

## 📡 Data Flow

### 1. Movement Control Flow
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

## 🛠️ Development

### Project Structure

```
ros2-robot-dashboard/
├── src/
│   ├── components/
│   │   ├── CameraFeed.tsx       # Live MJPEG video display component
│   │   ├── VirtualJoystick.tsx  # Touch/mouse joystick with nipplejs
│   │   ├── TelemetryPanel.tsx   # Real-time system metrics display
│   │   ├── ConnectionPanel.tsx  # ROS Bridge connection manager
│   │   └── ui/                  # shadcn/ui components
│   ├── pages/
│   │   └── Index.tsx            # Main dashboard page
│   ├── hooks/                   # Custom React hooks
│   ├── lib/
│   │   └── utils.ts             # Utility functions
│   ├── index.css                # Global styles + design tokens
│   └── main.tsx                 # App entry point
├── public/
│   └── robots.txt
├── index.html
├── tailwind.config.ts           # Tailwind configuration
├── vite.config.ts               # Vite build configuration
└── package.json
```

### Tech Stack

| Layer | Technology | Purpose |
|-------|-----------|---------|
| **Frontend Framework** | React 18.3 + TypeScript | UI components and type safety |
| **Build Tool** | Vite | Fast development and optimized builds |
| **Styling** | Tailwind CSS + shadcn/ui | Utility-first CSS and component library |
| **ROS Integration** | roslib.js | WebSocket communication with ROS Bridge |
| **Joystick** | nipplejs | Virtual joystick with touch/mouse support |
| **State Management** | React Hooks (useState, useEffect, useCallback) | Component state and lifecycle |
| **Notifications** | Sonner | Toast notifications for connection status |

### Design System

The dashboard implements a cohesive design system with semantic color tokens:

- **Theme:** Dark mode optimized for low-light robotics environments
- **Primary Colors:** Cyan/blue accent colors (#06b6d4) for interactive elements
- **Typography:** System font stack with clear hierarchy
- **Layout:** Responsive CSS Grid (mobile-first approach)
- **Components:** Glassmorphism effects with backdrop blur
- **Animations:** Smooth transitions and real-time status indicators

### Key Dependencies

```json
{
  "roslib": "^1.4.1",           // ROS Bridge WebSocket client
  "nipplejs": "^0.10.2",        // Virtual joystick library
  "lucide-react": "^0.462.0",   // Icon library
  "sonner": "^1.7.4"            // Toast notifications
}
```

## 🐛 Troubleshooting

### Connection Issues

**Problem:** "Failed to connect to ROS Bridge"
- ✅ Check Raspberry Pi is on the same network
- ✅ Verify rosbridge_server is running: `ros2 node list | grep rosbridge`
- ✅ Test WebSocket: `ws://<PI_IP>:9090` in browser console
- ✅ Check firewall: `sudo ufw allow 9090`

**Problem:** Camera feed not showing
- ✅ Verify web_video_server is running: `ros2 node list | grep web_video_server`
- ✅ Test stream URL directly in browser: `http://<PI_IP>:8080/stream?topic=/camera/image_raw`
- ✅ Check camera is detected: `v4l2-ctl --list-devices`
- ✅ Verify camera node is publishing: `ros2 topic hz /camera/image_raw`

**Problem:** Robot not responding to joystick
- ✅ Check connection status indicator shows "Connected"
- ✅ Verify cmd_vel topic: `ros2 topic echo /cmd_vel`
- ✅ Check motor driver power supply
- ✅ Test pigpio daemon: `sudo systemctl status pigpiod`

### Performance Optimization

**Laggy video feed:**
```bash
# Reduce camera resolution in camera_node.py
self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```

**High CPU usage on Pi:**
```bash
# Lower camera frame rate
self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz instead of 20 Hz
```

## 🚀 Deployment

### Production Build

```bash
npm run build
```

This creates an optimized production build in the `dist/` folder.

### Hosting Options

1. **Lovable (Recommended):** Click "Publish" in the Lovable editor
2. **GitHub Pages:** Push to `gh-pages` branch
3. **Vercel/Netlify:** Connect your GitHub repo for automatic deployments
4. **Self-hosted:** Serve the `dist/` folder with nginx or Apache

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is open source and available under the MIT License.

## 🙏 Acknowledgments

- Built with [Lovable](https://lovable.dev)
- Powered by ROS 2
- Icons from [Lucide](https://lucide.dev)

## 📞 Support

- **Documentation:** https://docs.lovable.dev
- **Project URL:** https://lovable.dev/projects/f0be7543-b124-4377-ad5a-5a8d78771707
- **Discord:** [Lovable Community](https://discord.com/channels/1119885301872070706/1280461670979993613)

---

Made with ❤️ using Lovable

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
