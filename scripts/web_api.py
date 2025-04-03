from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# To run server manually:
# uvicorn web_api:app --host mcalec.dyn.wpi.edu --port 8000
# To test, run this on another computer:
# curl -X POST http://mcalec.dyn.wpi.edu:8000/send_command/ -H "Content-Type: application/json" -d '{"command": "test"}'
# Additional installations:
# pip install 'uvicorn[standard]' websockets wsproto asyncio

# Initialize FastAPI
app = FastAPI()

# Allow all origins (Django UI)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Change this later for security
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# @app.websocket("/ws/status/")
# async def websocket_endpoint(websocket: WebSocket):
#     await websocket.accept()
#     print("✅ WebSocket Connected!")
#     try:
#         while True:
#             # Wait for messages, timeout if necessary
#             try:
#                 # data = await asyncio.wait_for(websocket.receive_text(), timeout=30)
#                 data = await websocket.receive_text()
#                 print(f"📩 Received: {data}")
#                 await websocket.send_text("✅ Acknowledged!") 
#             except asyncio.TimeoutError:
#                 print("⏳ No data received, keeping connection alive...")
#                 await websocket.send_text("🔄 Still connected")  # Keep connection alive
#             # except WebSocketDisconnect:
#             #     print("🔌 Client Disconnected")
#     except WebSocketDisconnect:
#         print("🔌 UI Disconnected")
#     except Exception as e:
#         print(f"❌ WebSocket Error: {e}")
#     finally:
#         print("🔒 Closing WebSocket")
        # await websocket.close()

@app.websocket("/ws/status/")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("✅ WebSocket Connected!")

    try:
        while True:
            data = await websocket.receive_text()  # Wait for messages
            print(f"📩 Received: {data}")

            await websocket.send_text("✅ Acknowledged!")  # Send response
            
    except WebSocketDisconnect:
        print("🔌 Client Disconnected")
    except Exception as e:
        print(f"❌ WebSocket Error: {e}")

    finally:
        print("🔒 Closing WebSocket")
        await websocket.close()

# Start ROS2 node
rclpy.init()

class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_web_bridge')
        self.publisher = self.create_publisher(String, '/game_command', 10)

    def send_command(self, command: str):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent command to ROS2: {command}")
        return {"status": "command sent", "command": command}

# Create ROS2 node
node = ROS2BridgeNode()

@app.post("/api/game/")
async def send_command(command: dict):
    robot_command = command.get("command", "default_command")  # Read command
    print(robot_command)
    return node.send_command(robot_command)  # Send it to ROS2


active_connections = {}


@app.websocket("/ws/game/{session_id}/")
async def websocket_endpoint(websocket: WebSocket, session_id: str):
    await websocket.accept()
    active_connections[session_id] = websocket
    print(f"✅ WebSocket connected for session {session_id}")

    try:
        while True:
            message = await websocket.receive_text()
            data = json.loads(message)

            # Process commands from UI
            if data["command"] == "detect_piece":
                piece = detect_piece(data["x"], data["y"])
                response = {
                    "type": "piece_detected",
                    "piece": piece
                }
                await websocket.send_text(json.dumps(response))

            elif data["command"] == "move_piece":
                success = move_piece(data["piece_id"], data["target_x"], data["target_y"])
                response = {
                    "type": "move_response",
                    "success": success
                }
                await websocket.send_text(json.dumps(response))

    except WebSocketDisconnect:
        print(f"🔌 Disconnected session {session_id}")
        del active_connections[session_id]

# Mock piece detection function
def detect_piece(x, y):
    return {"id": "piece_1", "type": "pawn", "x": x, "y": y}

# Mock piece movement function
def move_piece(piece_id, x, y):
    print(f"Moving piece {piece_id} to {x}, {y}")
    return True  # Simulate success