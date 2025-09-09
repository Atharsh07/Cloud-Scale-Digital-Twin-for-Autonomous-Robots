import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rl_agent.agent import RLAgent
import numpy as np

class RLNavigationNode(Node):
    def __init__(self):
        super().__init__('rl_navigation_node')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.agent = RLAgent(actions=['forward', 'left', 'right', 'stop'])
        self.state = np.array([1.0]*360)  # dummy initial state
        self.get_logger().info("RL Navigation Node Started")

    def scan_callback(self, msg):
        # Convert LaserScan to state representation
        self.state = np.array(msg.ranges)
        action = self.agent.choose_action(self.state)
        reward = self.compute_reward(msg.ranges)
        next_state = self.state
        self.agent.update_q(self.state, action, reward, next_state)
        self.state = next_state
        self.execute_action(action)

    def compute_reward(self, ranges):
        # simple reward: closer to obstacles = negative reward
        min_dist = min(ranges)
        if min_dist < 0.2:
            return -10.0
        return 1.0

    def execute_action(self, action):
        cmd = Twist()
        if action == 'forward':
            cmd.linear.x = 0.2
        elif action == 'left':
            cmd.angular.z = 0.5
        elif action == 'right':
            cmd.angular.z = -0.5
        elif action == 'stop':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RLNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
