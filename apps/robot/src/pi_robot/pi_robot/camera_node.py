import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ROS 2 Parameters
        self.declare_parameter('frame_rate', 20.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        self.frame_rate = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        # Publisher for /camera/image_raw
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        # Timer for capturing and publishing frames
        self.timer = self.create_timer(1.0 / self.frame_rate, self.timer_callback)

        # OpenCV setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video capture device.")
            rclpy.shutdown()
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS 2 Image message
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "camera"

            self.publisher_.publish(ros_image_msg)
        else:
            self.get_logger().warn("Could not read frame from camera.")

    def on_shutdown(self):
        self.get_logger().info("Shutting down camera_node, releasing camera.")
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.on_shutdown()
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
