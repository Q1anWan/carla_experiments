"""RGB camera recorder for synchronous CARLA runs."""

from __future__ import annotations

import queue
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Callable, Optional

import carla

from ..config import CameraConfig
from ..utils import ensure_dir


TickCallback = Callable[[carla.WorldSnapshot, carla.Image, int], None]


@dataclass
class CameraRecorder:
    world: carla.World
    ego_vehicle: carla.Actor
    config: CameraConfig

    _camera: Optional[carla.Sensor] = None
    _queue: Optional["queue.Queue[carla.Image]"] = None

    def start(self) -> None:
        blueprint = self.world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(self.config.width))
        blueprint.set_attribute("image_size_y", str(self.config.height))
        blueprint.set_attribute("fov", str(self.config.fov))
        # In sync mode, let the world tick drive the sensor capture to avoid dropped frames.
        blueprint.set_attribute("sensor_tick", "0.0")

        transform = self._resolve_transform()
        self._queue = queue.Queue()
        self._camera = self.world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle)
        self._camera.listen(self._queue.put)

    def stop(self) -> None:
        if self._camera is None:
            return
        try:
            self._camera.stop()
        except RuntimeError:
            pass
        try:
            self._camera.destroy()
        except RuntimeError:
            pass
        self._camera = None
        self._queue = None

    def record_frames(
        self,
        frames_dir: Path,
        num_frames: int,
        *,
        timeout: float = 5.0,
        on_tick: Optional[TickCallback] = None,
    ) -> None:
        if self._camera is None or self._queue is None:
            raise RuntimeError("Recorder not started. Call start() first.")
        ensure_dir(frames_dir)

        for index in range(num_frames):
            frame = self.world.tick()
            snapshot = self.world.get_snapshot()
            image = self._get_image(frame, timeout)
            image.save_to_disk(str(frames_dir / f"{index:06d}.png"))
            if on_tick:
                on_tick(snapshot, image, index)

    def _get_image(self, frame: int, timeout: float) -> carla.Image:
        assert self._queue is not None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                image = self._queue.get(timeout=remaining)
            except queue.Empty:
                continue
            if image.frame == frame:
                return image
        raise RuntimeError(
            f"Timed out waiting for camera frame {frame}. "
            "Check sensor spawn and sync settings."
        )

    def _resolve_transform(self) -> carla.Transform:
        preset = (self.config.preset or "").lower()
        if preset and preset not in {"custom", "manual"}:
            return self._transform_from_preset(preset)
        return carla.Transform(
            carla.Location(
                x=float(self.config.transform.get("x", 0.8)),
                y=float(self.config.transform.get("y", 0.0)),
                z=float(self.config.transform.get("z", 1.3)),
            ),
            carla.Rotation(
                pitch=float(self.config.transform.get("pitch", 0.0)),
                yaw=float(self.config.transform.get("yaw", 0.0)),
                roll=float(self.config.transform.get("roll", 0.0)),
            ),
        )

    def _transform_from_preset(self, preset: str) -> carla.Transform:
        bound_x = 0.5 + self.ego_vehicle.bounding_box.extent.x
        bound_y = 0.5 + self.ego_vehicle.bounding_box.extent.y
        bound_z = 0.5 + self.ego_vehicle.bounding_box.extent.z
        if preset == "driver":
            return carla.Transform(
                carla.Location(x=0.8 * bound_x, y=0.0 * bound_y, z=1.3 * bound_z),
                carla.Rotation(pitch=0.0),
            )
        if preset in {"third_person", "chase"}:
            return carla.Transform(
                carla.Location(x=-2.0 * bound_x, y=0.0 * bound_y, z=2.0 * bound_z),
                carla.Rotation(pitch=8.0),
            )
        if preset in {"hood", "bonnet"}:
            return carla.Transform(
                carla.Location(x=1.4 * bound_x, y=0.0 * bound_y, z=1.2 * bound_z),
                carla.Rotation(pitch=0.0),
            )
        raise ValueError(f"Unknown camera preset: {preset}")


def record_video(
    scenario_ctx: "ScenarioContext",
    out_dir: Path,
    *,
    on_tick: Optional[TickCallback] = None,
) -> Path:
    frames_dir = out_dir / "frames"
    recorder = CameraRecorder(
        world=scenario_ctx.world,
        ego_vehicle=scenario_ctx.ego_vehicle,
        config=scenario_ctx.camera_config,
    )
    recorder.start()
    try:
        num_frames = int(scenario_ctx.duration * scenario_ctx.fps)
        recorder.record_frames(frames_dir, num_frames, on_tick=on_tick)
    finally:
        recorder.stop()
    return frames_dir


from ..scenarios.base import ScenarioContext  # noqa: E402  (deferred import)
