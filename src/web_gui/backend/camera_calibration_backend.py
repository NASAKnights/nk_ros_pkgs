import asyncio
import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from fastapi import APIRouter, Request, HTTPException
from fastapi.responses import StreamingResponse

from v4l2_camera.action import CalibrateCamera
from calibration_state import feedback_queues

router = APIRouter()

calibration_client = None
current_goal_handle = None


class CalibrationClient(Node):
    def __init__(self, client_name):
        super().__init__('calibration_client')
        self.client = ActionClient(self, CalibrateCamera, client_name)

    def send_goal(self, goal_msg):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Calibration server not available.")
            return False

        self.get_logger().info("Sending goal...")
        
        self._send_goal_future = self.client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)

        self._send_goal_future.add_done_callback(self.goal_done)
        return True

    def goal_done(self, fut):
        global current_goal_handle
        goal_handle = fut.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Calibration goal rejected")
            return
        self.get_logger().info("Goal accepted")
        current_goal_handle = goal_handle
    
    def feedback_cb(self, msg):
        # Convert potentially non-serializable fields to plain floats
        def safe_float(val):
            try:
                return float(val[0]) if hasattr(val, '__getitem__') else float(val)
            except Exception:
                return 0.0

        feedback = {
            "accepted": msg.feedback.accepted,
            "reason": msg.feedback.reason,
            "frames_captured": msg.feedback.frames_captured,
            "area": safe_float(msg.feedback.area_diversity),
            "x_offset": safe_float(msg.feedback.x_percent),
            "y_offset": safe_float(msg.feedback.y_percent),
            "skew_score": safe_float(msg.feedback.skew_diversity),
        }

        print("Feedback:", feedback)

        for q in feedback_queues:
            q.put_nowait(json.dumps(feedback))

import threading

spin_thread = None

def ensure_client(client_name):
    global calibration_client, spin_thread

    if calibration_client is None:
        if not rclpy.ok():
            rclpy.init(args=None)

        calibration_client = CalibrationClient(client_name)

        # Start spinning the node in a daemon thread
        spin_thread = threading.Thread(
            target=rclpy.spin,
            args=(calibration_client,),
            daemon=True
        )
        spin_thread.start()

    return calibration_client


@router.get("/api/calibrate/feedback")
async def calibration_feedback():
    queue = asyncio.Queue()
    feedback_queues.append(queue)
    print("Frontend connected to feedback stream")

    async def event_stream():
        try:
            while True:
                data = await queue.get()
                yield f"data: {data}\n\n"
        except asyncio.CancelledError:
            print("Feedback stream closed by frontend")
        finally:
            feedback_queues.remove(queue)

    return StreamingResponse(event_stream(), media_type="text/event-stream")


@router.post("/api/calibrate/intrinsic")
async def start_calibration(request: Request):
    body = await request.json()

    # Required fields from frontend
    namespace = body.get("namespace")
    
    if not namespace:
        raise HTTPException(status_code=400, detail="Missing required 'namespace' field")
    
    board_type = body.get("pattern", "chessboard")
    rows = int(body.get("rows", 6))
    cols = int(body.get("cols", 9))
    square_size = float(body.get("square_size", 0.024))

    goal = CalibrateCamera.Goal()
    goal.board_type = board_type
    goal.rows = rows
    goal.cols = cols
    goal.square_size = square_size

    # Ensure the action client is for the correct namespaced action
    action_name = f"{namespace}/calibrate_camera"
    client = ensure_client(action_name)

    if client.send_goal(goal):
        return {"message": f"Calibration started on {action_name}"}
    else:
        return HTTPException(status_code=400, detail="Can't send calibration goal")


@router.post("/api/calibrate/cancel")
async def cancel_calibration():
    global current_goal_handle, calibration_client
    if current_goal_handle is not None:
        future = current_goal_handle.cancel_goal_async()
        if calibration_client is not None:
            rclpy.spin_until_future_complete(calibration_client, future)
        current_goal_handle = None
        return {"message": "Calibration cancelled"}
    return HTTPException(status_code=400, detail="No active calibration to cancel")
    
