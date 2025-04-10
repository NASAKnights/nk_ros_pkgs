# backend/calibration_state.py
from typing import List
from fastapi.responses import StreamingResponse
from asyncio import Queue

feedback_queues: List[Queue] = []
