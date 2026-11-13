"""HTTP bridge between the GCS and the full_sim flood mission runner."""

from __future__ import annotations

import copy
import math
import os
import sys
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from fastapi import APIRouter, File, Form, HTTPException, UploadFile
from fastapi.responses import Response

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_KML_PATH = PROJECT_ROOT / "frontend" / "public" / "mock_arena_geofence.kml"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

router = APIRouter()
EARTH_RADIUS_M = 6_371_000.0
SIM_MAP_ORIGIN_XY = (20.0, 0.0)
SIM_DRONE_START_XY = (20.0, -13.0)
# The synthetic 40 x 30 m world is normalized to the fixed KML's 30 x 40 m
# east-west/north-south bounds so the full survey loop stays inside the arena.
SIM_X_TO_KML_WIDTH = 30.0 / 40.0
SIM_Y_TO_KML_LENGTH = 40.0 / 30.0


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def parse_kml_home(raw: bytes) -> tuple[float, float, float]:
    """Use the first KML Point as launch site, else the geometry centroid."""
    if not raw or len(raw) > 10 * 1024 * 1024:
        raise ValueError("Upload a KML file smaller than 10 MB")
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise ValueError("Could not read this KML file") from exc

    points: list[tuple[float, float, float]] = []
    all_coords: list[tuple[float, float, float]] = []
    coordinate_groups: list[list[tuple[float, float, float]]] = []
    for element in root.iter():
        if _local_name(element.tag) != "coordinates" or not element.text:
            continue
        parsed = []
        for token in element.text.replace("\n", " ").split():
            parts = token.split(",")
            if len(parts) < 2:
                continue
            try:
                lon, lat = float(parts[0]), float(parts[1])
                alt = float(parts[2]) if len(parts) > 2 and parts[2] else 0.0
            except ValueError:
                continue
            if -180 <= lon <= 180 and -90 <= lat <= 90:
                parsed.append((lat, lon, alt))
                all_coords.append((lat, lon, alt))
        parent_is_point = False
        for parent in root.iter():
            if element in list(parent):
                parent_is_point = _local_name(parent.tag) == "Point"
                break
        if parent_is_point and parsed:
            points.extend(parsed)
        elif parsed:
            coordinate_groups.append(parsed)

    if points:
        return points[0]
    if not all_coords:
        raise ValueError("KML needs at least one Point, LineString, or Polygon coordinate")
    outer_boundary = max(
        coordinate_groups,
        key=lambda group: (max(item[0] for item in group) - min(item[0] for item in group))
        * (max(item[1] for item in group) - min(item[1] for item in group)),
        default=all_coords,
    )
    return (
        (min(item[0] for item in outer_boundary) + max(item[0] for item in outer_boundary)) / 2,
        (min(item[1] for item in outer_boundary) + max(item[1] for item in outer_boundary)) / 2,
        sum(item[2] for item in outer_boundary) / len(outer_boundary),
    )


def local_to_geo(x: float, y: float, home: tuple[float, float, float]) -> tuple[float, float]:
    lat0, lon0, _ = home
    lat = lat0 + math.degrees((y - SIM_MAP_ORIGIN_XY[1]) * SIM_Y_TO_KML_LENGTH / EARTH_RADIUS_M)
    cosine = max(1e-6, math.cos(math.radians(lat0)))
    lon = lon0 + math.degrees((x - SIM_MAP_ORIGIN_XY[0]) * SIM_X_TO_KML_WIDTH / (EARTH_RADIUS_M * cosine))
    return lat, lon


def _jpeg(image: np.ndarray) -> bytes:
    bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 82])
    if not ok:
        raise RuntimeError("Could not encode simulation camera frame")
    return bytes(encoded)


@dataclass
class FloodSession:
    session_id: str
    home: tuple[float, float, float]
    backend: str
    status: str = "starting"
    state: dict[str, Any] = field(default_factory=dict)
    planned_route: list[dict[str, float]] = field(default_factory=list)
    trail: list[dict[str, float]] = field(default_factory=list)
    cameras: dict[str, bytes] = field(default_factory=dict)
    stop_event: threading.Event = field(default_factory=threading.Event)
    thread: threading.Thread | None = None
    output_dir: Path | None = None
    error: str | None = None
    last_trail_xy: tuple[float, float] | None = None


class FloodSimulationManager:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._sessions: dict[str, FloodSession] = {}
        self._active_id: str | None = None

    def start(self, kml: bytes, altitude: float, speed: float, realtime: bool) -> dict[str, Any]:
        if not 4 <= altitude <= 40 or not 0.2 <= speed <= 5:
            raise ValueError("Choose altitude 4–40 m and speed 0.2–5 m/s")
        arena_center = parse_kml_home(kml)
        launch_lat, launch_lon = local_to_geo(*SIM_DRONE_START_XY, arena_center)
        home = (launch_lat, launch_lon, arena_center[2])
        try:
            from full_sim import flood_mission, flood_world
            import pybullet  # noqa: F401 - fail before returning a session
        except Exception as exc:
            raise RuntimeError(f"Flood simulation dependencies are unavailable: {exc}") from exc

        with self._lock:
            if self._active_id:
                active = self._sessions.get(self._active_id)
                if active and active.status in {"starting", "running", "returning"}:
                    raise RuntimeError("A flood simulation is already active")

            session_id = str(uuid.uuid4())
            session = FloodSession(session_id=session_id, home=home, backend="pybullet")
            session.output_dir = Path(tempfile.mkdtemp(prefix=f"flood-gcs-{session_id[:8]}-"))
            for name, label in (("front", "FRONT CAMERA"), ("down", "DOWN CAMERA")):
                image = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(image, label, (16, 120), cv2.FONT_HERSHEY_SIMPLEX, .7, (220, 240, 235), 2)
                session.cameras[name] = _jpeg(image)

            route = flood_mission.survey_path(altitude)
            session.planned_route = []
            for x, y, z in route:
                lat, lon = local_to_geo(x, y, arena_center)
                session.planned_route.append({"latitude": lat, "longitude": lon, "altitude": z})
            self._sessions[session_id] = session
            self._active_id = session_id

            args = type("FloodMissionArgs", (), {})()
            args.gui = False
            args.headless = True
            args.realtime = realtime
            args.altitude = altitude
            args.speed = speed
            # Keep PyBullet's 60 Hz motion/collision step intact, but reduce
            # expensive camera rendering for small shared demo instances.
            args.fps = 2.5
            args.max_seconds = 1800.0
            args.no_video = True
            args.output = session.output_dir

            def publish_state(value: dict[str, Any]) -> None:
                pose = value.get("pose", {})
                x, y = float(pose.get("x", SIM_DRONE_START_XY[0])), float(pose.get("y", SIM_DRONE_START_XY[1]))
                lat, lon = local_to_geo(x, y, arena_center)
                people = []
                for person in value.get("people", []):
                    person_lat, person_lon = local_to_geo(float(person["x"]), float(person["y"]), arena_center)
                    people.append({**person, "latitude": person_lat, "longitude": person_lon})
                mapped = {
                    **value,
                    "drone": {"latitude": lat, "longitude": lon, "altitude": float(pose.get("z", 0.0))},
                    "people": people,
                }
                with self._lock:
                    if self._sessions.get(session_id) is not session:
                        return
                    if session.last_trail_xy is None or math.dist((x, y), session.last_trail_xy) >= 0.35:
                        session.trail.append({"latitude": lat, "longitude": lon, "altitude": float(pose.get("z", 0.0))})
                        session.last_trail_xy = (x, y)
                    mapped["trail"] = copy.deepcopy(session.trail)
                    session.state = mapped
                    session.status = "returning" if value.get("phase") == "RETURN" else "running"

            def publish_frames(down: np.ndarray, front: np.ndarray, _phase: str, _time_s: float) -> None:
                front_jpeg, down_jpeg = _jpeg(front), _jpeg(down)
                with self._lock:
                    if self._sessions.get(session_id) is session:
                        session.cameras["front"] = front_jpeg
                        session.cameras["down"] = down_jpeg

            def run_worker() -> None:
                session.status = "running"
                try:
                    result = flood_mission.run(
                        args,
                        on_state=publish_state,
                        on_frames=publish_frames,
                        stop_event=session.stop_event,
                    )
                    with self._lock:
                        if result == 0:
                            session.status = "complete"
                            session.state = {**session.state, "phase": "COMPLETE", "armed": False}
                        else:
                            session.status = "stopped"
                            session.state = {**session.state, "phase": "STOPPED"}
                except Exception as exc:
                    with self._lock:
                        session.status = "failed"
                        session.error = str(exc)
                        session.state = {**session.state, "phase": "ERROR", "error": str(exc)}

            session.thread = threading.Thread(target=run_worker, name=f"flood-{session_id[:8]}", daemon=True)
            session.thread.start()
            return {
                "session_id": session_id,
                "status": session.status,
                "backend": "pybullet",
                "home": {"latitude": home[0], "longitude": home[1], "altitude": home[2]},
                "planned_route": session.planned_route,
            }

    def get(self, session_id: str) -> FloodSession:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                raise KeyError(session_id)
            return session

    def active(self) -> dict[str, Any] | None:
        with self._lock:
            session = self._sessions.get(self._active_id) if self._active_id else None
            if session is None or session.status not in {"starting", "running", "returning"}:
                return None
            return {
                "session_id": session.session_id,
                "status": session.status,
                "backend": session.backend,
                "home": {"latitude": session.home[0], "longitude": session.home[1], "altitude": session.home[2]},
                "planned_route": copy.deepcopy(session.planned_route),
                "state": copy.deepcopy(session.state),
                "error": session.error,
            }

    def state(self, session_id: str) -> dict[str, Any]:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                raise KeyError(session_id)
            return {
                "session_id": session.session_id,
                "status": session.status,
                "backend": session.backend,
                "home": {"latitude": session.home[0], "longitude": session.home[1]},
                "planned_route": copy.deepcopy(session.planned_route),
                "state": copy.deepcopy(session.state),
                "error": session.error,
            }

    def stop(self, session_id: str) -> dict[str, str]:
        session = self.get(session_id)
        with self._lock:
            if session.status not in {"starting", "running", "returning"}:
                return {"session_id": session_id, "status": session.status}
            session.status = "returning"
            session.stop_event.set()
        return {"session_id": session_id, "status": "returning"}

    def camera(self, session_id: str, camera: str) -> bytes:
        if camera not in {"front", "down"}:
            raise ValueError("Camera must be front or down")
        return self.get(session_id).cameras[camera]


manager = FloodSimulationManager()


@router.post("/simulation/flood/start")
async def start_flood_simulation(
    kml_file: UploadFile | None = File(None),
    altitude: float = Form(5.0),
    speed: float = Form(3.0),
    realtime: bool = Form(True),
):
    try:
        if kml_file:
            kml_bytes = await kml_file.read()
        else:
            if not DEFAULT_KML_PATH.is_file():
                raise RuntimeError(f"Default flood mission KML is missing: {DEFAULT_KML_PATH}")
            kml_bytes = DEFAULT_KML_PATH.read_bytes()
        return manager.start(kml_bytes, altitude, speed, realtime)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc


@router.get("/simulation/flood/active")
def active_flood_simulation():
    return manager.active()


@router.get("/simulation/flood/{session_id}/state")
async def flood_simulation_state(session_id: str):
    try:
        return manager.state(session_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Simulation session not found") from exc


@router.get("/simulation/flood/{session_id}/camera/{camera}")
async def flood_simulation_camera(session_id: str, camera: str):
    try:
        image = manager.camera(session_id, camera)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Simulation session not found") from exc
    except ValueError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    return Response(content=image, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@router.post("/simulation/flood/{session_id}/stop")
async def stop_flood_simulation(session_id: str):
    try:
        return manager.stop(session_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Simulation session not found") from exc
