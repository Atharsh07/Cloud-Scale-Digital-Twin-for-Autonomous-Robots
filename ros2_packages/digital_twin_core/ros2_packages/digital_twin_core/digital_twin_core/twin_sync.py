
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String

class TwinSyncNode(Node):
    def __init__(self):
        super().__init__('twin_sync_node')
        self.pose_pub = self.create_publisher(Pose, '/virtual_robot/pose', 10)
        self.pose_sub = self.create_subscription(Pose, '/physical_robot/pose', self.pose_callback, 10)
        self.get_logger().info("Digital Twin Sync Node Started")

    def pose_callback(self, msg):
        # Forward physical robot pose to virtual twin
        self.pose_pub.publish(msg)
        self.get_logger().debug(f"Synced Pose: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = TwinSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
