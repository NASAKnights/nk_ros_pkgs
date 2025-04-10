import asyncio
import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from fastapi import APIRouter, Request
from fastapi.responses import StreamingResponse

from v4l2_camera.action import CalibrateCamera
from calibration_state import feedback_queues

router = APIRouter()

calibration_client = None
current_goal_handle = None


class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        self.client = ActionClient(self, CalibrateCamera, 'calibrate_camera')

    def send_goal(self, goal_msg, feedback_cb, done_cb):
        global current_goal_handle

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Calibration server not available.")
            return None

        future = self.client.send_goal_async(
            goal_msg, feedback_callback=feedback_cb)

        def _goal_done(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Calibration goal rejected")
                return
            current_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(done_cb)

        future.add_done_callback(_goal_done)
        return future


def ensure_client():
    global calibration_client
    if calibration_client is None:
        rclpy.init(args=None)
        calibration_client = CalibrationClient()
        rclpy.get_global_executor().add_node(calibration_client)
    return calibration_client


def feedback_cb(msg):
    feedback = {
        "accepted": msg.feedback.accepted,
        "reason": msg.feedback.reason,
        "frames_captured": msg.feedback.frames_captured,
        "area": msg.feedback.area,
        "x_offset": msg.feedback.x_offset,
        "y_offset": msg.feedback.y_offset,
        "skew_score": msg.feedback.skew_score,
    }
    print("Feedback:", feedback)
    for q in feedback_queues:
        q.put_nowait(json.dumps(feedback))


@router.get("/api/calibrate/feedback")
async def calibration_feedback():
    queue = asyncio.Queue()
    feedback_queues.append(queue)

    async def event_stream():
        try:
            while True:
                data = await queue.get()
                yield f"data: {data}\n\n"
        except asyncio.CancelledError:
            pass
        finally:
            feedback_queues.remove(queue)

    return StreamingResponse(event_stream(), media_type="text/event-stream")


@router.post("/api/calibrate/intrinsic")
async def start_calibration(request: Request):
    body = await request.json()
    client = ensure_client()

    goal = CalibrateCamera.Goal()
    goal.board_type = body.get("pattern", "chessboard")
    goal.rows = body.get("rows", 6)
    goal.cols = body.get("cols", 9)
    goal.square_size = float(body.get("square_size", 0.024))
    goal.num_frames = body.get("frames", 10)

    def done_cb(future):
        result = future.result().get_result()
        print("Calibration complete:", result)

    client.send_goal(goal, feedback_cb, done_cb)

    return {"message": "Calibration started"}


@router.post("/api/calibrate/cancel")
async def cancel_calibration():
    global current_goal_handle
    if current_goal_handle is not None:
        client = ensure_client()
        future = current_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(client, future)
        current_goal_handle = None
        return {"message": "Calibration cancelled"}
    return {"message": "No active calibration to cancel"}
