import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FleetManagerNode(Node):
    def __init__(self):
        super().__init__('fleet_manager_node')
        self.robot_status = {}  # {robot_id: status}
        self.status_sub = self.create_subscription(String, '/robot_status', self.status_callback, 10)
        self.get_logger().info("Fleet Manager Node Started")

    def status_callback(self, msg):
        robot_id, status = msg.data.split(',')
        self.robot_status[robot_id] = status
        self.get_logger().info(f"Updated {robot_id} status: {status}")
        self.optimize_fleet()

    def optimize_fleet(self):
        # Simple placeholder for resource optimization
        active_robots = [r for r, s in self.robot_status.items() if s == 'active']
        self.get_logger().debug(f"Active Robots: {active_robots}")

def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
