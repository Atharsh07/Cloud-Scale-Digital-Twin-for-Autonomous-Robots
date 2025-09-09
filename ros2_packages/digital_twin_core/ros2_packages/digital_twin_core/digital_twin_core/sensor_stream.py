import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import asyncio
import websockets
import json

class SensorStreamNode(Node):
    def __init__(self):
        super().__init__('sensor_stream_node')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.clients = set()
        self.get_logger().info("Sensor Stream Node Started")
        asyncio.ensure_future(self.websocket_server())

    async def websocket_server(self):
        async with websockets.serve(self.handle_client, '0.0.0.0', 8765):
            await asyncio.Future()  # run forever

    async def handle_client(self, websocket, path):
        self.clients.add(websocket)
        self.get_logger().info(f"Client connected: {websocket.remote_address}")
        try:
            await websocket.wait_closed()
        finally:
            self.clients.remove(websocket)

    def scan_callback(self, msg):
        data = {
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'ranges': msg.ranges
        }
        asyncio.ensure_future(self.broadcast(json.dumps(data)))

    async def broadcast(self, message):
        if self.clients:
            await asyncio.wait([client.send(message) for client in self.clients])

def main(args=None):
    rclpy.init(args=args)
    node = SensorStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
