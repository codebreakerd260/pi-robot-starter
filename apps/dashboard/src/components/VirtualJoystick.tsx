import { useEffect, useRef } from "react";
import { Card } from "@/components/ui/card";
// @ts-ignore
import nipplejs from "nipplejs";

interface VirtualJoystickProps {
  onMove: (linear: number, angular: number) => void;
  onStop: () => void;
  isConnected: boolean;
}

export const VirtualJoystick = ({
  onMove,
  onStop,
  isConnected,
}: VirtualJoystickProps) => {
  const joystickRef = useRef<HTMLDivElement>(null);
  const managerRef = useRef<any>(null);

  useEffect(() => {
    if (!joystickRef.current || !isConnected) return;

    managerRef.current = nipplejs.create({
      zone: joystickRef.current,
      mode: "static",
      position: { left: "50%", top: "50%" },
      color: "hsl(var(--primary))",
      size: 150,
    });

    managerRef.current.on("move", (_evt: any, data: any) => {
      const linear = data.vector.y * 0.6;
      const angular = -data.vector.x * 1.0;
      onMove(linear, angular);
    });

    managerRef.current.on("end", () => {
      onStop();
    });

    return () => {
      if (managerRef.current) {
        managerRef.current.destroy();
      }
    };
  }, [isConnected, onMove, onStop]);

  return (
    <Card className="border-border/50 bg-card/50 backdrop-blur-sm">
      <div className="p-6">
        <h3 className="text-sm font-semibold mb-4 text-foreground">
          Robot Control
        </h3>
        <div
          ref={joystickRef}
          className={`w-full h-64 rounded-lg bg-muted/30 border border-border/30 relative ${
            !isConnected ? "opacity-50" : ""
          }`}
        >
          {!isConnected && (
            <div className="absolute inset-0 flex items-center justify-center text-muted-foreground text-sm">
              Connect to enable control
            </div>
          )}
        </div>
        <div className="mt-4 grid grid-cols-2 gap-2 text-xs text-muted-foreground">
          <div className="flex items-center gap-2">
            <div className="w-2 h-2 rounded-full bg-primary" />
            <span>Up/Down: Forward/Backward</span>
          </div>
          <div className="flex items-center gap-2">
            <div className="w-2 h-2 rounded-full bg-primary" />
            <span>Left/Right: Turn</span>
          </div>
        </div>
      </div>
    </Card>
  );
};
