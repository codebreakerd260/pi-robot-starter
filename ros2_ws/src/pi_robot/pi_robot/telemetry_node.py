import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json
import time

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        self.publisher_ = self.create_publisher(String, '/telemetry', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # 1 Hz
        self.start_time = time.time()

    def get_cpu_temperature(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = int(f.read()) / 1000.0
            return temp
        except FileNotFoundError:
            return None # Not all systems have this file

    def timer_callback(self):
        uptime_seconds = time.time() - self.start_time

        telemetry_data = {
            'cpu_temp': self.get_cpu_temperature(),
            'cpu_usage': psutil.cpu_percent(),
            'mem_usage': psutil.virtual_memory().percent,
            'uptime': uptime_seconds
        }

        msg = String()
        msg.data = json.dumps(telemetry_data)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    telemetry_node = TelemetryNode()
    try:
        rclpy.spin(telemetry_node)
    except KeyboardInterrupt:
        pass
    finally:
        telemetry_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
