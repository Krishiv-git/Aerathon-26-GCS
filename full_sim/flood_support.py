"""Small PyBullet adapters shared by the browser flood mission."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Sequence

import numpy as np
import pybullet as p

COLLISION_LENGTH_M = 0.588
COLLISION_WIDTH_M = 0.588
COLLISION_HEIGHT_M = 0.335
BODY_LENGTH_M = 0.383
BODY_WIDTH_M = 0.343
BODY_HEIGHT_M = 0.335
TOTAL_MASS_KG = 1.85
MOTOR_DIAGONAL_M = 0.472
PROPELLER_DIAMETER_M = 0.254
MOTOR_AXIS_SPAN_M = MOTOR_DIAGONAL_M / math.sqrt(2)
GROUND_CENTER_Z = COLLISION_HEIGHT_M / 2.0 + 0.01

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_VFOV_DEG = 48.0
CAMERA_FORWARD_OFFSET_M = 0.15


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float


class MissionFailure(RuntimeError):
    """Raised when a simulated flight command cannot be completed."""


class KinematicVehicle:
    """Deterministic flight adapter used by the PyBullet flood simulation."""

    def __init__(self, body_id: int, start: Sequence[float], yaw: float, latency_s: float = 0.0):
        self.body_id = body_id
        self.latency_s = latency_s
        self.sim_time_s = 0.0
        self._x, self._y, self._z = map(float, start)
        self._yaw = float(yaw)
        self._mode = "DISARMED"
        self._armed = False
        self._takeoff_target: float | None = None
        self._applied_velocity = (0.0, 0.0, 0.0)
        self._queue: deque[tuple[float, tuple[float, float, float]]] = deque()
        self._sync_bullet_pose()

    @property
    def sim_time(self) -> float:
        return self.sim_time_s

    @property
    def armed(self) -> bool:
        return self._armed

    def arm(self) -> None:
        self._armed = True
        self._mode = "GUIDED"

    def set_yaw(self, yaw: float) -> None:
        self._yaw = float(yaw)
        self._sync_bullet_pose()

    def takeoff(self, altitude_m: float) -> None:
        if not self._armed:
            raise MissionFailure("Takeoff requested before arming")
        self._queue.clear()
        self._applied_velocity = (0.0, 0.0, 0.0)
        self._takeoff_target = float(altitude_m)
        self._mode = "TAKEOFF"

    def send_velocity_body(self, vx: float, vy: float, vz: float = 0.0) -> None:
        if self._mode in {"DISARMED", "LAND", "LANDED"}:
            return
        delivery_time = self.sim_time_s + self.latency_s
        self._queue.append((delivery_time, (float(vx), float(vy), float(vz))))

    def flush_velocity_commands(self) -> None:
        self._queue.clear()
        self._applied_velocity = (0.0, 0.0, 0.0)

    def land(self) -> None:
        self._queue.clear()
        self._applied_velocity = (0.0, 0.0, 0.0)
        self._takeoff_target = None
        self._mode = "LAND"

    def step(self, dt_s: float) -> None:
        dt_s = max(0.0, float(dt_s))
        self.sim_time_s += dt_s
        while self._queue and self._queue[0][0] <= self.sim_time_s + 1e-9:
            _, self._applied_velocity = self._queue.popleft()

        if self._mode == "TAKEOFF" and self._takeoff_target is not None:
            error = self._takeoff_target - self._z
            climb = max(-1.5, min(1.5, 1.5 * error))
            if abs(error) < 0.02:
                self._z = self._takeoff_target
                self._takeoff_target = None
                self._mode = "GUIDED"
            else:
                self._z += climb * dt_s
        elif self._mode == "LAND":
            self._z = max(GROUND_CENTER_Z, self._z - 1.0 * dt_s)
            if self._z <= GROUND_CENTER_Z + 0.01:
                self._z = GROUND_CENTER_Z
                self._mode = "LANDED"
                self._armed = False
        elif self._mode == "GUIDED":
            vx, vy, vz = self._applied_velocity
            cos_yaw, sin_yaw = math.cos(self._yaw), math.sin(self._yaw)
            self._x += (cos_yaw * vx - sin_yaw * vy) * dt_s
            self._y += (sin_yaw * vx + cos_yaw * vy) * dt_s
            self._z = max(GROUND_CENTER_Z, self._z - vz * dt_s)
        self._sync_bullet_pose()

    def get_pose(self) -> Pose:
        return Pose(self._x, self._y, self._z, self._yaw)

    def _sync_bullet_pose(self) -> None:
        if p.isConnected():
            p.resetBasePositionAndOrientation(
                self.body_id,
                [self._x, self._y, self._z],
                p.getQuaternionFromEuler([0.0, 0.0, self._yaw]),
            )

    def sync_sim_pose(self) -> None:
        self._sync_bullet_pose()

    def close(self) -> None:
        self._queue.clear()


class SimulationCamera:
    """Render the drone's downward-facing simulated camera."""

    def __init__(self, body_id: int):
        self.body_id = body_id

    def render_downward(self) -> np.ndarray:
        if not p.isConnected():
            return np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)

        pos, ori = p.getBasePositionAndOrientation(self.body_id)
        rotation = np.asarray(p.getMatrixFromQuaternion(ori), dtype=float).reshape(3, 3)
        position = np.asarray(pos, dtype=float)
        eye = position + rotation @ np.array([CAMERA_FORWARD_OFFSET_M, 0.0, -0.10])
        target = position + rotation @ np.array([CAMERA_FORWARD_OFFSET_M, 0.0, -8.0])
        up = rotation @ np.array([1.0, 0.0, 0.0])
        view = p.computeViewMatrix(eye.tolist(), target.tolist(), up.tolist())
        projection = p.computeProjectionMatrixFOV(
            CAMERA_VFOV_DEG,
            CAMERA_WIDTH / CAMERA_HEIGHT,
            0.05,
            50.0,
        )
        method = p.getConnectionInfo().get("connectionMethod")
        renderer = p.ER_TINY_RENDERER if method == p.DIRECT else p.ER_BULLET_HARDWARE_OPENGL
        _, _, rgba, _, _ = p.getCameraImage(
            CAMERA_WIDTH,
            CAMERA_HEIGHT,
            view,
            projection,
            renderer=renderer,
        )
        return np.asarray(rgba, dtype=np.uint8).reshape(CAMERA_HEIGHT, CAMERA_WIDTH, 4)[:, :, :3]


class DroneVisual:
    """Simple collidable quadcopter mesh for the simulation view and camera."""

    def __init__(self, start_pos: Sequence[float], start_yaw: float = 0.0):
        half = (COLLISION_LENGTH_M / 2.0, COLLISION_WIDTH_M / 2.0, COLLISION_HEIGHT_M / 2.0)
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        offset = MOTOR_AXIS_SPAN_M / 2.0
        rotor_pos = [(x * offset, y * offset, 0.07) for x, y in [(1, 1), (1, -1), (-1, 1), (-1, -1)]]
        visual = p.createVisualShapeArray(
            shapeTypes=[p.GEOM_BOX] + [p.GEOM_CYLINDER] * 4,
            halfExtents=[(BODY_LENGTH_M / 2.0, BODY_WIDTH_M / 2.0, BODY_HEIGHT_M / 2.0)] + [(0, 0, 0)] * 4,
            radii=[0] + [PROPELLER_DIAMETER_M / 2.0] * 4,
            lengths=[0] + [0.006] * 4,
            visualFramePositions=[(0, 0, 0)] + rotor_pos,
            rgbaColors=[(0.15, 0.25, 0.80, 0.65)] + [(0.12, 0.12, 0.12, 0.80)] * 4,
        )
        self.body_id = p.createMultiBody(
            TOTAL_MASS_KG,
            collision,
            visual,
            start_pos,
            p.getQuaternionFromEuler((0, 0, start_yaw)),
        )
