import { useState, useEffect, useCallback } from "react";
import { CameraFeed } from "@/components/CameraFeed";
import { VirtualJoystick } from "@/components/VirtualJoystick";
import { TelemetryPanel } from "@/components/TelemetryPanel";
import { ConnectionPanel } from "@/components/ConnectionPanel";
import { Bot } from "lucide-react";
import { toast } from "sonner";
// @ts-ignore
import ROSLIB from "roslib";

interface TelemetryData {
  cpu_temp?: number;
  cpu?: number;
  mem?: number;
  uptime?: number;
}

const Index = () => {
  const [isConnected, setIsConnected] = useState(false);
  const [ros, setRos] = useState<any>(null);
  const [cmdVelTopic, setCmdVelTopic] = useState<any>(null);
  const [telemetryData, setTelemetryData] = useState<TelemetryData | null>(null);
  const [streamUrl, setStreamUrl] = useState("");

  const handleConnect = useCallback((rosUrl: string, cameraUrl: string) => {
    try {
      const rosInstance = new ROSLIB.Ros({
        url: rosUrl,
      });

      rosInstance.on("connection", () => {
        setIsConnected(true);
        toast.success("Connected to robot", {
          description: "ROS Bridge connection established",
        });

        // Setup cmd_vel topic
        const cmdVel = new ROSLIB.Topic({
          ros: rosInstance,
          name: "/cmd_vel",
          messageType: "geometry_msgs/Twist",
        });
        setCmdVelTopic(cmdVel);

        // Setup telemetry subscription
        const telemetry = new ROSLIB.Topic({
          ros: rosInstance,
          name: "/telemetry",
          messageType: "std_msgs/String",
        });

        telemetry.subscribe((message: any) => {
          try {
            const data = JSON.parse(message.data);
            setTelemetryData(data);
          } catch (e) {
            console.error("Failed to parse telemetry data", e);
          }
        });
      });

      rosInstance.on("error", (error: any) => {
        console.error("ROS connection error:", error);
        toast.error("Connection error", {
          description: "Failed to connect to ROS Bridge",
        });
        setIsConnected(false);
      });

      rosInstance.on("close", () => {
        setIsConnected(false);
        toast.info("Disconnected from robot");
      });

      setRos(rosInstance);
      setStreamUrl(cameraUrl);
    } catch (error) {
      console.error("Failed to initialize ROS connection:", error);
      toast.error("Connection failed", {
        description: "Could not initialize ROS connection",
      });
    }
  }, []);

  const handleDisconnect = useCallback(() => {
    if (ros) {
      ros.close();
      setRos(null);
      setCmdVelTopic(null);
      setIsConnected(false);
      setTelemetryData(null);
    }
  }, [ros]);

  const handleJoystickMove = useCallback(
    (linear: number, angular: number) => {
      if (cmdVelTopic && isConnected) {
        const twist = new ROSLIB.Message({
          linear: { x: linear, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: angular },
        });
        cmdVelTopic.publish(twist);
      }
    },
    [cmdVelTopic, isConnected]
  );

  const handleJoystickStop = useCallback(() => {
    if (cmdVelTopic && isConnected) {
      const twist = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      cmdVelTopic.publish(twist);
    }
  }, [cmdVelTopic, isConnected]);

  return (
    <div className="min-h-screen bg-background p-4 md:p-6 lg:p-8">
      <div className="max-w-7xl mx-auto space-y-6">
        {/* Header */}
        <div className="flex items-center gap-4 mb-8">
          <div className="p-3 rounded-xl bg-primary/10 border border-primary/20">
            <Bot className="w-8 h-8 text-primary" />
          </div>
          <div>
            <h1 className="text-3xl font-bold text-foreground">ROS 2 Robot Dashboard</h1>
            <p className="text-sm text-muted-foreground">
              Control your Raspberry Pi robot in real-time
            </p>
          </div>
        </div>

        {/* Main Grid */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          {/* Left Column - Connection & Telemetry */}
          <div className="space-y-6">
            <ConnectionPanel
              isConnected={isConnected}
              onConnect={handleConnect}
              onDisconnect={handleDisconnect}
            />
            <TelemetryPanel data={telemetryData} isConnected={isConnected} />
          </div>

          {/* Middle Column - Camera Feed */}
          <div className="lg:col-span-2 space-y-6">
            <CameraFeed streamUrl={streamUrl} isConnected={isConnected} />
            <VirtualJoystick
              onMove={handleJoystickMove}
              onStop={handleJoystickStop}
              isConnected={isConnected}
            />
          </div>
        </div>

        {/* Footer */}
        <div className="text-center text-xs text-muted-foreground pt-6 border-t border-border/30">
          <p>
            Powered by ROS 2 • Make sure rosbridge_server and web_video_server are running on your
            robot
          </p>
        </div>
      </div>
    </div>
  );
};

export default Index;
