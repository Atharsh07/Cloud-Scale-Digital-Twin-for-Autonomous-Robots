import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class TaskSchedulerNode(Node):
    def __init__(self):
        super().__init__('task_scheduler_node')
        self.task_pub = self.create_publisher(String, '/assign_task', 10)
        self.robot_list = ['robot1', 'robot2', 'robot3']
        self.tasks = ['delivery', 'inspection', 'mapping']
        self.get_logger().info("Task Scheduler Node Started")
        self.create_timer(5.0, self.assign_task)  # Assign every 5 seconds

    def assign_task(self):
        robot_id = random.choice(self.robot_list)
        task = random.choice(self.tasks)
        msg = String()
        msg.data = f"{robot_id},{task}"
        self.task_pub.publish(msg)
        self.get_logger().info(f"Assigned task '{task}' to {robot_id}")

def main(args=None):
    rclpy.init(args=args)
    node = TaskSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
