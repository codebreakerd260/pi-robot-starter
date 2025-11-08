import { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Camera } from "lucide-react";

interface CameraFeedProps {
  streamUrl: string;
  isConnected: boolean;
}

export const CameraFeed = ({ streamUrl, isConnected }: CameraFeedProps) => {
  const [hasError, setHasError] = useState(false);

  useEffect(() => {
    setHasError(false);
  }, [streamUrl]);

  return (
    <Card className="relative overflow-hidden border-border/50 bg-card/50 backdrop-blur-sm">
      <div className="aspect-video w-full bg-muted/30 flex items-center justify-center relative">
        {isConnected && !hasError ? (
          <>
            <img
              src={streamUrl}
              alt="Robot camera feed"
              className="w-full h-full object-cover"
              onError={() => setHasError(true)}
            />
            <div className="absolute top-3 left-3 flex items-center gap-2 bg-background/80 backdrop-blur-sm px-3 py-1.5 rounded-full border border-border/50">
              <div className="w-2 h-2 rounded-full bg-destructive animate-pulse shadow-lg shadow-destructive/50" />
              <span className="text-xs font-medium text-foreground">LIVE</span>
            </div>
          </>
        ) : (
          <div className="flex flex-col items-center gap-3 text-muted-foreground">
            <Camera className="w-12 h-12 opacity-50" />
            <p className="text-sm">
              {!isConnected ? "Waiting for connection..." : "Camera feed unavailable"}
            </p>
          </div>
        )}
      </div>
    </Card>
  );
};
