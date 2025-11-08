# Web Dashboard

This directory contains the web-based dashboard for the ROS 2 robot. It is a real-time control interface built with React, TypeScript, and Vite, and it communicates with the robot through a ROS Bridge connection. For a complete overview of the project, see the [main `README.md`](../../README.md).

## Tech Stack

| Layer                  | Technology                                     | Purpose                                   |
| ---------------------- | ---------------------------------------------- | ----------------------------------------- |
| **Frontend Framework** | React 18.3 + TypeScript                        | UI components and type safety             |
| **Build Tool**         | Vite                                           | Fast development and optimized builds     |
| **Styling**            | Tailwind CSS + shadcn/ui                       | Utility-first CSS and component library   |
| **ROS Integration**    | roslib.js                                      | WebSocket communication with ROS Bridge   |
| **Joystick**           | nipplejs                                       | Virtual joystick with touch/mouse support |
| **State Management**   | React Hooks (useState, useEffect, useCallback) | Component state and lifecycle             |
| **Notifications**      | Sonner                                         | Toast notifications for connection status |

## Project Structure

```
dashboard/
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

## Development

To run the dashboard in a local development environment, follow these steps:

1.  **Install dependencies:**
    ```bash
    npm install
    ```
2.  **Start the development server:**
    ```bash
    npm run dev
    ```
3.  **Open the dashboard:**
    Navigate to `http://localhost:5173` in your browser.

## Deployment

### Production Build

To create an optimized production build, run the following command:

```bash
npm run build
```

This will generate a `dist/` directory with the compiled and minified assets.

### Hosting

The `dist/` directory can be deployed to any static hosting service, such as:

-   **Lovable (Recommended):** Click "Publish" in the Lovable editor
-   **GitHub Pages:** Push to `gh-pages` branch
-   **Vercel/Netlify:** Connect your GitHub repo for automatic deployments
-   **Self-hosted:** Serve the `dist/` folder with nginx or Apache
