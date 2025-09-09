import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class AnomalyDetectionNode(Node):
    def __init__(self):
        super().__init__('anomaly_detection_node')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("Anomaly Detection Node Started")

    def scan_callback(self, msg):
        threshold = 0.2  # meters
        if any(distance < threshold for distance in msg.ranges):
            self.get_logger().warn("Obstacle detected! Potential anomaly!")

def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
