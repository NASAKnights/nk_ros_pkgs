# backend/main.py
from fastapi import FastAPI, HTTPException, Request, Query
from pathlib import Path
import psutil
import subprocess
import time
import platform
import shutil
from typing import Literal, Optional
from fastapi.middleware.cors import CORSMiddleware
import subprocess
from pydantic import BaseModel
from typing import List
import subprocess
from subprocess import Popen
from typing import Optional
import re
import os
from pathlib import Path
from config import router as config_router
from camera_calibration_backend import router as calibration_router

app = FastAPI()
web_video_process: Optional[Popen] = None
web_video_topic: Optional[str] = None

# Allow local frontend access
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)
app.include_router(config_router)
app.include_router(calibration_router)


@app.get("/api/nodes")
def get_nodes():
    result = subprocess.run(["ros2", "node", "list"],
                            capture_output=True, text=True)
    return {"nodes": result.stdout.strip().split("\n")}


@app.get("/api/topics")
def get_topics():
    result = subprocess.run(["ros2", "topic", "list"],
                            capture_output=True, text=True)
    return {"topics": result.stdout.strip().split("\n")}


@app.get("/api/camera_topics")
def list_camera_topics() -> dict:
    result = subprocess.run(
        ["ros2", "topic", "list", "-t"],
        capture_output=True,
        text=True
    )
    if result.returncode != 0:
        raise HTTPException(status_code=500, detail="Failed to list topics")

    image_topics: List[str] = []
    for line in result.stdout.strip().splitlines():
        if "[" in line and "]" in line:
            topic, msg_type = line.split("[")
            topic = topic.strip()
            msg_type = msg_type.strip(" ]")

            if msg_type in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"):
                image_topics.append(topic)
    print(image_topics)
    return {"topics": image_topics}


@app.get("/api/system")
def get_system_info():
    system_name = platform.uname().machine.lower()
    is_jetson = "tegra" in system_name or "aarch64" in system_name

    if is_jetson:
        temps = get_jetson_temperatures()
    else:
        temps = []
        try:
            temps_raw = psutil.sensors_temperatures(fahrenheit=False)
            for label, entries in temps_raw.items():
                for entry in entries:
                    if entry.current is not None:
                        temps.append({
                            "label": label,
                            "sensor": entry.label or "unnamed",
                            "temp": round(entry.current, 1)
                        })
        except Exception as e:
            print(f"Warning: Failed to read psutil temperatures: {e}")

    return {
        "platform": platform.system(),
        "platform_release": platform.release(),
        "cpu_percent": psutil.cpu_percent(interval=0.5),
        "cpu_cores": psutil.cpu_count(logical=False),
        "cpu_threads": psutil.cpu_count(logical=True),
        "ram_used": psutil.virtual_memory().used // 1024**2,
        "ram_total": psutil.virtual_memory().total // 1024**2,
        "disk_used": shutil.disk_usage("/").used // 1024**3,
        "disk_total": shutil.disk_usage("/").total // 1024**3,
        "uptime_seconds": int(time.time() - psutil.boot_time()),
        "temps": temps
    }


def get_jetson_temperatures():
    temp_entries = []

    thermal_zone_base = Path("/sys/class/thermal")
    zones = list(thermal_zone_base.glob("thermal_zone*/temp"))

    for temp_file in zones:
        try:
            zone_path = temp_file.parent
            type_path = zone_path / "type"

            # Read label
            label = type_path.read_text().strip() if type_path.exists() else "unknown"

            # Read temperature
            raw_temp = temp_file.read_text().strip()
            if raw_temp:
                temp_c = float(raw_temp) / 1000.0
                temp_entries.append({
                    "label": label,
                    "sensor": zone_path.name,
                    "temp": round(temp_c, 1)
                })
        except Exception as e:
            print(f"Failed to read {temp_file}: {e}")

    return temp_entries


@app.post("/api/camera/start")
async def start_camera_stream(request: Request):
    global web_video_process, web_video_topic

    # Stop existing process if any
    if web_video_process is not None:
        web_video_process.terminate()
        web_video_process.wait()
        web_video_process = None
        web_video_topic = None

    # Start new web_video_server with remapping
    try:
        web_video_process = subprocess.Popen([
            "ros2", "run", "web_video_server", "web_video_server",
            "--ros-args", "-p", "port:=8081"
        ])
        return {"status": "started"}
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to start stream: {e}")


@app.post("/api/camera/stop")
def stop_camera_stream():
    global web_video_process, web_video_topic

    if web_video_process is not None:
        web_video_process.terminate()
        web_video_process.wait()
        web_video_process = None
        web_video_topic = None
        return {"status": "stopped"}
    else:
        return {"status": "not running"}


class CalibrateRequest(BaseModel):
    pattern: Literal["chessboard", "charuco"] = "chessboard"
    square_size: float
    rows: int
    cols: int
    camera_topic: str
    output_file: str = "calibration.yaml"

    # Optional for ChArUco
    charuco_dict: Optional[str] = "DICT_5X5_1000"
    charuco_marker_length: Optional[float] = None


@app.post("/api/calibrate/intrinsic")
def run_intrinsic_calibration(data: CalibrateRequest):

    cmd = [
        "ros2", "run", "v4l2_camera", "calibration_node",
        "--ros-args",
        "-p", f"pattern_type:={data.pattern}",
        "-p", f"capture_topic:={data.camera_topic}",
        "-p", f"output_file:={data.output_file}",
    ]

    if data.pattern == "chessboard":
        cmd += [
            "-p", f"chessboard_rows:={data.rows}",
            "-p", f"chessboard_cols:={data.cols}",
            "-p", f"square_size:={data.square_size}",
        ]
    elif data.pattern == "charuco":
        marker_len = data.charuco_marker_length or data.square_size / 2
        cmd += [
            "-p", f"charuco_squares_x:={data.cols}",
            "-p", f"charuco_squares_y:={data.rows}",
            "-p", f"charuco_square_length:={data.square_size}",
            "-p", f"charuco_marker_length:={marker_len}",
            "-p", f"charuco_dict:={data.charuco_dict}",
        ]

    print("Launching calibration command:", " ".join(cmd))
    subprocess.Popen(cmd)
    return {"message": f"Calibration process launched for {data.camera_topic}"}


@app.get("/api/nodes/introspect")
def introspect_node(package: str, executable: str):
    try:
        result = subprocess.run(
            ["ros2", "run", package, executable,
                "--ros-args", "--print-arguments"],
            capture_output=True, text=True, timeout=5
        )

        output = result.stdout
        param_matches = re.findall(r"-p (\w+):", output)
        remap_matches = re.findall(r"--remap (\w+):=", output)

        return {
            "parameters": list(set(param_matches)),
            "remaps": list(set(remap_matches)),
        }

    except Exception as e:
        return {"parameters": [], "remaps": [], "error": str(e)}


@app.get("/api/ros/packages")
def list_ros_packages():
    try:
        result = subprocess.run(["ros2", "pkg", "list"],
                                capture_output=True, text=True, check=True)
        return result.stdout.strip().split("\n")
    except subprocess.CalledProcessError as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to list packages: {e}")


@app.get("/api/ros/executables")
def list_ros_executables(package: str):
    try:
        result = subprocess.run(
            ["ros2", "pkg", "executables", package],
            capture_output=True, text=True, check=True
        )
        # Format: "<package> <executable>"
        lines = result.stdout.strip().split("\n")
        return [line.split()[1] for line in lines if line.startswith(package)]
    except subprocess.CalledProcessError as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to list executables: {e}")


@app.get("/api/ros/launch_files")
def get_launch_files(package: str = Query(...)):
    try:
        # Get package share directory
        result = subprocess.run(
            ["ros2", "pkg", "prefix", package],
            capture_output=True, text=True, check=True
        )
        base_path = Path(result.stdout.strip()) / "share" / package

        if not base_path.exists():
            raise HTTPException(
                status_code=404, detail="Package share path not found")

        launch_files = []
        for root, _, files in os.walk(base_path):
            for file in files:
                if file.endswith(".launch.py") or file.endswith(".launch.xml"):
                    rel_path = str(Path(file))
                    launch_files.append(rel_path)

        return launch_files

    except subprocess.CalledProcessError:
        raise HTTPException(
            status_code=404, detail="Failed to find package path")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/ros/launch_args")
def get_launch_args(package: str, file: str):
    import re

    try:
        result = subprocess.run(
            ["ros2", "launch", package, file, "--show-args"],
            capture_output=True, text=True, check=True
        )

        lines = result.stdout.strip().splitlines()
        args = []
        i = 0

        while i < len(lines):
            line = lines[i].strip()
            if not line or line.startswith("Arguments"):
                i += 1
                continue

            # Match: 'arg_name':
            match = re.match(r"'(.+)':", line)
            if match:
                name = match.group(1)
                description = ""
                default = ""

                # Next line: description
                if i + 1 < len(lines):
                    description = lines[i + 1].strip()

                # Line after that: default
                if i + 2 < len(lines):
                    default_match = re.match(
                        r"\(default:\s*'(.+)'\)", lines[i + 2].strip())
                    if default_match:
                        default = default_match.group(1)

                args.append({
                    "name": name,
                    "description": description,
                    "default": default
                })

                i += 3  # move to next argument block
            else:
                i += 1
        print(args)
        return args

    except subprocess.CalledProcessError as e:
        raise HTTPException(
            status_code=500, detail=e.stderr or "Failed to parse launch args")
