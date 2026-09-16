#!/usr/bin/env python3
"""Generate drone camera POV frames and publish their URLs as Yamcs local parameters."""

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw


WIDTH = 960
HEIGHT = 540
SUPERSAMPLE = 2


@dataclass(frozen=True)
class DroneState:
    second: int
    mode: int
    phase: str
    armed: bool
    failsafe: bool
    altitude_m: float
    speed_mps: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    north_m: float
    east_m: float


def flight_state(second: int) -> tuple[int, str, bool, float, float]:
    t = second % 300
    if t < 30:
        return 0, "STANDBY", False, 0.0, 0.0
    if t < 60:
        progress = (t - 30) / 30
        return 2, "TAKEOFF", True, 40.0 * progress, 3.0 * progress
    if t < 180:
        return 3, "SURVEY", True, 40.0 + 2.0 * math.sin(t / 12), 12.0
    if t < 220:
        progress = (t - 180) / 40
        return 4, "RETURN", True, 40.0 - 5.0 * progress, 14.0
    if t < 250:
        progress = (t - 220) / 30
        return 5, "LANDING", True, 35.0 * (1.0 - progress), 4.0 * (1.0 - progress)
    return 6, "CHARGING", False, 0.0, 0.0


def mission_position(second: int) -> tuple[float, float]:
    t = second % 300
    if t < 30:
        return 0.0, 0.0
    if t < 60:
        progress = (t - 30) / 30
        return 8.0 * progress, 5.0 * progress
    if t < 180:
        a = (t - 60) / 120 * 2 * math.pi
        return 120.0 * math.sin(a), 65.0 * math.sin(2 * a)
    if t < 220:
        progress = (t - 180) / 40
        return 0.0, 0.0 * (1.0 - progress)
    if t < 250:
        progress = (t - 220) / 30
        return 3.0 * (1.0 - progress), 2.0 * (1.0 - progress)
    return 0.0, 0.0


def drone_state(second: int) -> DroneState:
    mode, phase, armed, altitude_m, speed_mps = flight_state(second)
    failsafe = 190 <= second % 300 < 205
    if failsafe:
        mode = 4
        phase = "FAILSAFE_RETURN"

    in_flight = armed and mode not in (0, 1, 6)
    roll_deg = 4.0 * math.sin(second / 8.0) if in_flight else 0.0
    pitch_deg = 3.0 * math.sin(second / 10.0) if in_flight else 0.0
    yaw_deg = (second * 1.7) % 360
    north_m, east_m = mission_position(second)

    return DroneState(
        second=second,
        mode=mode,
        phase=phase,
        armed=armed,
        failsafe=failsafe,
        altitude_m=altitude_m,
        speed_mps=speed_mps,
        roll_deg=roll_deg,
        pitch_deg=pitch_deg,
        yaw_deg=yaw_deg,
        north_m=north_m,
        east_m=east_m,
    )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def lerp(a: int, b: int, t: float) -> int:
    return round(a + (b - a) * clamp(t, 0.0, 1.0))


def blend(a: tuple[int, int, int], b: tuple[int, int, int], t: float) -> tuple[int, int, int]:
    return lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t)


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def camera_basis(state: DroneState) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    yaw = math.radians(state.yaw_deg)
    pitch = math.radians(state.pitch_deg - 12.0)
    roll = math.radians(state.roll_deg)

    forward = (
        math.cos(pitch) * math.cos(yaw),
        math.cos(pitch) * math.sin(yaw),
        math.sin(pitch),
    )
    world_right = (-math.sin(yaw), math.cos(yaw), 0.0)
    world_up = (
        -math.sin(pitch) * math.cos(yaw),
        -math.sin(pitch) * math.sin(yaw),
        math.cos(pitch),
    )

    right = (
        world_right[0] * math.cos(roll) + world_up[0] * math.sin(roll),
        world_right[1] * math.cos(roll) + world_up[1] * math.sin(roll),
        world_right[2] * math.cos(roll) + world_up[2] * math.sin(roll),
    )
    up = (
        world_up[0] * math.cos(roll) - world_right[0] * math.sin(roll),
        world_up[1] * math.cos(roll) - world_right[1] * math.sin(roll),
        world_up[2] * math.cos(roll) - world_right[2] * math.sin(roll),
    )
    return forward, right, up


def project_ground_point(
    state: DroneState,
    north_m: float,
    east_m: float,
    width: int,
    height: int,
) -> tuple[float, float] | None:
    camera_z = max(1.8, state.altitude_m + 1.0)
    vector = (north_m - state.north_m, east_m - state.east_m, -camera_z)
    forward, right, up = camera_basis(state)

    depth = dot(vector, forward)
    if depth <= 1.0:
        return None

    x = dot(vector, right)
    y = dot(vector, up)
    focal = width * 0.82
    return width / 2 + focal * x / depth, height / 2 - focal * y / depth


def draw_sky(draw: ImageDraw.ImageDraw, width: int, height: int, state: DroneState) -> None:
    top = (72, 150, 225)
    horizon = (183, 220, 245)
    if state.failsafe:
        top = (98, 130, 190)
        horizon = (225, 190, 170)

    for y in range(height):
        draw.line([(0, y), (width, y)], fill=blend(top, horizon, y / height))


def ground_horizon_y(state: DroneState, height: int) -> int:
    # Camera looks slightly down by default. Pitch moves the horizon naturally;
    # altitude gives a slightly wider visible ground region.
    pitch_effect = (state.pitch_deg - 12.0) / 36.0
    altitude_effect = clamp(state.altitude_m / 80.0, 0.0, 1.0) * 0.08
    return round(height * clamp(0.48 + pitch_effect - altitude_effect, 0.22, 0.72))


def draw_ground(draw: ImageDraw.ImageDraw, width: int, height: int, state: DroneState) -> None:
    horizon = ground_horizon_y(state, height)
    near = (76, 116, 82)
    far = (156, 181, 143)
    if state.failsafe:
        near = (104, 103, 86)
        far = (186, 170, 132)

    for y in range(horizon, height):
        t = (y - horizon) / max(1, height - horizon)
        draw.line([(0, y), (width, y)], fill=blend(far, near, t))


def visible_polyline(points: list[tuple[float, float] | None], width: int, height: int) -> list[tuple[float, float]]:
    visible: list[tuple[float, float]] = []
    margin = max(width, height) * 2
    for point in points:
        if point is None:
            if len(visible) >= 2:
                break
            visible = []
            continue
        x, y = point
        if -margin <= x <= width + margin and -margin <= y <= height + margin:
            visible.append((x, y))
    return visible


def draw_grid(draw: ImageDraw.ImageDraw, width: int, height: int, state: DroneState) -> None:
    yaw = math.radians(state.yaw_deg)
    forward = (math.cos(yaw), math.sin(yaw))
    right = (-math.sin(yaw), math.cos(yaw))
    grid_color = (214, 232, 204) if not state.failsafe else (232, 204, 176)

    spacing = 20.0
    max_depth = 620.0
    max_lateral = 360.0

    # Cross lines.
    for depth in [i * spacing for i in range(1, int(max_depth // spacing) + 1)]:
        points: list[tuple[float, float] | None] = []
        for step in range(-36, 37):
            lateral = step * spacing / 2
            north = state.north_m + forward[0] * depth + right[0] * lateral
            east = state.east_m + forward[1] * depth + right[1] * lateral
            points.append(project_ground_point(state, north, east, width, height))
        line = visible_polyline(points, width, height)
        if len(line) >= 2:
            alpha = clamp(1.0 - depth / max_depth, 0.15, 0.85)
            draw.line(line, fill=blend((120, 160, 126), grid_color, alpha), width=1)

    # Longitudinal lines.
    for lateral in [i * spacing for i in range(-int(max_lateral // spacing), int(max_lateral // spacing) + 1)]:
        points = []
        for step in range(1, int(max_depth // spacing) + 1):
            depth = step * spacing
            north = state.north_m + forward[0] * depth + right[0] * lateral
            east = state.east_m + forward[1] * depth + right[1] * lateral
            points.append(project_ground_point(state, north, east, width, height))
        line = visible_polyline(points, width, height)
        if len(line) >= 2:
            center_weight = clamp(1.0 - abs(lateral) / max_lateral, 0.2, 1.0)
            color = blend((104, 145, 112), grid_color, center_weight * 0.85)
            draw.line(line, fill=color, width=1)


def draw_lens_edges(draw: ImageDraw.ImageDraw, width: int, height: int) -> None:
    for i in range(42):
        shade = round(18 * (1.0 - i / 42))
        draw.rectangle((i, i, width - i - 1, height - i - 1), outline=(shade, shade + 5, shade + 9))


def render_frame(path: Path, state: DroneState) -> None:
    width = WIDTH * SUPERSAMPLE
    height = HEIGHT * SUPERSAMPLE
    image = Image.new("RGB", (width, height))
    draw = ImageDraw.Draw(image)

    draw_sky(draw, width, height, state)
    draw_ground(draw, width, height, state)
    draw_grid(draw, width, height, state)
    draw_lens_edges(draw, width, height)

    image = image.resize((WIDTH, HEIGHT), Image.Resampling.LANCZOS)
    image.save(path, optimize=True)


def connect_processor(yamcs_host: str, instance: str, processor: str, username: str | None, password: str | None):
    from yamcs.client import Credentials, YamcsClient

    try:
        client = YamcsClient(yamcs_host)
        proc = client.get_processor(instance=instance, processor=processor)
        print("Connected to Yamcs without authentication", flush=True)
        return proc
    except Exception as exc:
        if not username:
            raise RuntimeError(f"could not connect to Yamcs without authentication: {exc}") from exc
        credentials = Credentials(username=username, password=password or "")
        client = YamcsClient(yamcs_host, credentials=credentials)
        print(f"Connected to Yamcs as {username}", flush=True)
        return client.get_processor(instance=instance, processor=processor)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yamcs-host", default=os.getenv("YAMCS_HOST", "localhost:8090"))
    parser.add_argument("--public-yamcs-url", default=os.getenv("PUBLIC_YAMCS_URL", "http://localhost:8090"))
    parser.add_argument("--instance", default=os.getenv("YAMCS_INSTANCE", "demo"))
    parser.add_argument("--processor", default=os.getenv("YAMCS_PROCESSOR", "realtime"))
    parser.add_argument("--image-dir", type=Path, default=Path(os.getenv("IMAGE_DIR", "/tmp/images")))
    parser.add_argument("--interval", type=float, default=float(os.getenv("IMAGE_INTERVAL", "2")))
    parser.add_argument("--retain", type=int, default=int(os.getenv("IMAGE_RETAIN", "200")))
    parser.add_argument("--username", default=os.getenv("YAMCS_USERNAME"))
    parser.add_argument("--password", default=os.getenv("YAMCS_PASSWORD"))
    parser.add_argument("--render-only", type=Path, help="render one sample frame without connecting to Yamcs")
    parser.add_argument("--once", action="store_true", help="generate one frame and exit")
    args = parser.parse_args()

    args.image_dir.mkdir(parents=True, exist_ok=True)
    if args.render_only:
        render_frame(args.render_only, drone_state(96))
        print(f"rendered {args.render_only}")
        return

    processor = connect_processor(args.yamcs_host, args.instance, args.processor, args.username, args.password)

    frame = 1
    while True:
        state = drone_state(frame)
        image_name = f"drone_camera_{frame:05d}.png"
        image_path = args.image_dir / image_name
        render_frame(image_path, state)

        storage_url = f"/storage/buckets/images/objects/{image_name}"
        public_url = f"{args.public_yamcs_url.rstrip('/')}/api{storage_url}"
        processor.set_parameter_values(
            {
                "/drone/CameraImageNumber": frame,
                "/drone/CameraImageStorageUrl": storage_url,
                "/drone/CameraImageUrl": public_url,
                "/drone/CameraView": "forward-grid",
            }
        )

        if args.retain > 0:
            old_frame = frame - args.retain
            if old_frame > 0:
                (args.image_dir / f"drone_camera_{old_frame:05d}.png").unlink(missing_ok=True)

        print(f"published {public_url}", flush=True)

        if args.once:
            return
        frame += 1
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
