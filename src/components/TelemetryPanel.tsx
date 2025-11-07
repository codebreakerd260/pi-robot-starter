import { Card } from "@/components/ui/card";
import { Cpu, Thermometer, HardDrive, Clock } from "lucide-react";

interface TelemetryData {
  cpu_temp?: number;
  cpu?: number;
  mem?: number;
  uptime?: number;
}

interface TelemetryPanelProps {
  data: TelemetryData | null;
  isConnected: boolean;
}

export const TelemetryPanel = ({ data, isConnected }: TelemetryPanelProps) => {
  const formatUptime = (seconds: number) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    return `${hours}h ${minutes}m`;
  };

  const getStatusColor = (value: number, thresholds: { warning: number; danger: number }) => {
    if (value >= thresholds.danger) return "text-destructive";
    if (value >= thresholds.warning) return "text-warning";
    return "text-success";
  };

  const metrics = [
    {
      icon: Thermometer,
      label: "CPU Temp",
      value: data?.cpu_temp ? `${data.cpu_temp.toFixed(1)}°C` : "--",
      color: data?.cpu_temp
        ? getStatusColor(data.cpu_temp, { warning: 70, danger: 80 })
        : "text-muted-foreground",
    },
    {
      icon: Cpu,
      label: "CPU Usage",
      value: data?.cpu ? `${data.cpu.toFixed(1)}%` : "--",
      color: data?.cpu
        ? getStatusColor(data.cpu, { warning: 70, danger: 90 })
        : "text-muted-foreground",
    },
    {
      icon: HardDrive,
      label: "Memory",
      value: data?.mem ? `${data.mem.toFixed(1)}%` : "--",
      color: data?.mem
        ? getStatusColor(data.mem, { warning: 70, danger: 90 })
        : "text-muted-foreground",
    },
    {
      icon: Clock,
      label: "Uptime",
      value: data?.uptime ? formatUptime(data.uptime) : "--",
      color: "text-primary",
    },
  ];

  return (
    <Card className="border-border/50 bg-card/50 backdrop-blur-sm">
      <div className="p-6">
        <h3 className="text-sm font-semibold mb-4 text-foreground">System Telemetry</h3>
        <div className="grid grid-cols-2 gap-4">
          {metrics.map((metric, index) => (
            <div
              key={index}
              className="p-4 rounded-lg bg-muted/20 border border-border/30 hover:border-border/50 transition-colors"
            >
              <div className="flex items-center gap-3 mb-2">
                <metric.icon className={`w-4 h-4 ${metric.color}`} />
                <span className="text-xs text-muted-foreground">{metric.label}</span>
              </div>
              <p className={`text-2xl font-bold ${isConnected ? metric.color : "text-muted-foreground"}`}>
                {isConnected ? metric.value : "--"}
              </p>
            </div>
          ))}
        </div>
      </div>
    </Card>
  );
};
