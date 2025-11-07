import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Wifi, WifiOff } from "lucide-react";

interface ConnectionPanelProps {
  isConnected: boolean;
  onConnect: (rosUrl: string, streamUrl: string) => void;
  onDisconnect: () => void;
}

export const ConnectionPanel = ({ isConnected, onConnect, onDisconnect }: ConnectionPanelProps) => {
  const [rosUrl, setRosUrl] = useState("ws://192.168.1.100:9090");
  const [streamUrl, setStreamUrl] = useState("http://192.168.1.100:8080/stream?topic=/camera/image_raw");

  const handleConnect = () => {
    onConnect(rosUrl, streamUrl);
  };

  return (
    <Card className="border-border/50 bg-card/50 backdrop-blur-sm">
      <div className="p-6">
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-sm font-semibold text-foreground">Connection Settings</h3>
          <div className="flex items-center gap-2">
            {isConnected ? (
              <>
                <Wifi className="w-4 h-4 text-success" />
                <span className="text-xs font-medium text-success">Connected</span>
              </>
            ) : (
              <>
                <WifiOff className="w-4 h-4 text-muted-foreground" />
                <span className="text-xs font-medium text-muted-foreground">Disconnected</span>
              </>
            )}
          </div>
        </div>

        {!isConnected ? (
          <div className="space-y-3">
            <div>
              <label className="text-xs text-muted-foreground mb-1.5 block">ROS Bridge URL</label>
              <Input
                value={rosUrl}
                onChange={(e) => setRosUrl(e.target.value)}
                placeholder="ws://192.168.1.100:9090"
                className="bg-muted/30 border-border/30"
              />
            </div>
            <div>
              <label className="text-xs text-muted-foreground mb-1.5 block">Camera Stream URL</label>
              <Input
                value={streamUrl}
                onChange={(e) => setStreamUrl(e.target.value)}
                placeholder="http://192.168.1.100:8080/stream?topic=/camera/image_raw"
                className="bg-muted/30 border-border/30"
              />
            </div>
            <Button onClick={handleConnect} className="w-full shadow-lg shadow-primary/20">
              Connect to Robot
            </Button>
          </div>
        ) : (
          <Button onClick={onDisconnect} variant="destructive" className="w-full">
            Disconnect
          </Button>
        )}
      </div>
    </Card>
  );
};
