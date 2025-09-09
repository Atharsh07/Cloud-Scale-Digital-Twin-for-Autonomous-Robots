from flask import Flask, jsonify
from flask_cors import CORS
import asyncio
import websockets
import threading

app = Flask(__name__)
CORS(app)

# Store latest sensor data and robot status
sensor_data = {}
robot_status = {}

# WebSocket client to receive data from ROS2 sensor_stream
async def ws_client():
    uri = "ws://localhost:8765"  # From ROS2 sensor_stream node
    async with websockets.connect(uri) as websocket:
        async for message in websocket:
            global sensor_data
            sensor_data = message  # store latest sensor data

def start_ws_client():
    asyncio.new_event_loop().run_until_complete(ws_client())

# Start WebSocket client in a separate thread
threading.Thread(target=start_ws_client, daemon=True).start()

@app.route("/api/sensor")
def get_sensor_data():
    return jsonify(sensor_data)

@app.route("/api/status")
def get_robot_status():
    return jsonify(robot_status)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
