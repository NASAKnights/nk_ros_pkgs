from fastapi import HTTPException, Request, APIRouter
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from pydantic import BaseModel
import os
from pathlib import Path

import os
from fastapi import APIRouter
import logging

# Determine workspace root
DEFAULT_WORKSPACE = "/workspaces/ros_ws"
FALLBACK_WORKSPACE = os.path.expanduser("~/.ros")

workspace_root = os.environ.get("ROS_WORKSPACE", DEFAULT_WORKSPACE)
config_root = os.path.join(workspace_root, ".config")

# Ensure .config folder exists (with fallback if needed)
try:
    os.makedirs(config_root, exist_ok=True)
except PermissionError:
    logging.warning(
        f"Permission denied creating {config_root}, falling back to home directory.")
    workspace_root = FALLBACK_WORKSPACE
    config_root = os.path.join(workspace_root, ".config")
    os.makedirs(config_root, exist_ok=True)

logging.info(f"Using config directory: {config_root}")

# Export router
router = APIRouter()

# Optional: export CONFIG_ROOT if needed elsewhere
CONFIG_ROOT = config_root


@router.get("/api/configs/list")
def list_configs(path: str = ""):
    full_path = Path(CONFIG_ROOT) / path
    if not full_path.exists():
        raise HTTPException(status_code=404, detail="Path does not exist")
    entries = []
    for p in full_path.iterdir():
        entries.append({"name": p.name, "is_dir": p.is_dir()})
    return {"entries": entries}


@router.get("/api/configs/load")
def load_config(path: str):
    full_path = Path(CONFIG_ROOT) / path
    if not full_path.exists() or not full_path.is_file():
        raise HTTPException(status_code=404, detail="File not found")
    return {"content": full_path.read_text()}


@router.post("/api/configs/save")
async def save_config(request: Request):
    data = await request.json()
    full_path = Path(CONFIG_ROOT) / data["path"]
    full_path.parent.mkdir(parents=True, exist_ok=True)
    full_path.write_text(data["content"])
    return {"status": "ok"}

PRIMARY_FILE = Path(CONFIG_ROOT) / "primary_config.txt"


@router.get("/api/configs/primary")
def get_primary_config():
    if PRIMARY_FILE.exists():
        return {"path": PRIMARY_FILE.read_text().strip()}
    return {"path": None}


@router.post("/api/configs/primary")
async def set_primary_config(request: Request):
    data = await request.json()
    PRIMARY_FILE.write_text(data["path"])
    return {"status": "ok"}


CONFIG_ROOT = Path(get_package_share_directory("nk_vision")).resolve()


class SaveFileRequest(BaseModel):
    path: str
    content: str


@router.get("/api/configs/list")
def list_files(path: str = ""):
    full_path = (CONFIG_ROOT / Path(path)).resolve()
    if not full_path.is_dir() or CONFIG_ROOT not in full_path.parents and full_path != CONFIG_ROOT:
        raise HTTPException(status_code=400, detail="Invalid path")

    entries = []
    for entry in sorted(full_path.iterdir()):
        entries.append({
            "name": entry.name,
            "is_dir": entry.is_dir()
        })
    return {
        "path": str(full_path.relative_to(CONFIG_ROOT)),
        "entries": entries
    }


@router.get("/api/configs/load")
def load_file(path: str):
    full_path = (CONFIG_ROOT / Path(path)).resolve()
    if not full_path.is_file() or CONFIG_ROOT not in full_path.parents:
        raise HTTPException(status_code=400, detail="Invalid file")
    return {"content": full_path.read_text()}


@router.post("/api/configs/save")
def save_file(req: SaveFileRequest):
    full_path = (CONFIG_ROOT / Path(req.path)).resolve()
    if CONFIG_ROOT not in full_path.parents:
        raise HTTPException(status_code=400, detail="Invalid save path")
    full_path.write_text(req.content)
    return {"message": "Saved"}
