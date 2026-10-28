#!/usr/bin/env python3
"""Render the drone's forward camera and publish frame URLs as Yamcs local parameters.

The camera pose (position, altitude, roll, pitch, heading) is read from live Yamcs
telemetry each frame, so the view always matches what the dashboards show,
including commanded manoeuvres.

Rendering is a vectorised voxel-space raycaster: a tileable heightmap with baked
hillshade and cast shadows is generated once, then each frame casts one ray per
screen column and resolves the visible terrain sample for every pixel with a
per-column depth search. A frame costs a few tens of milliseconds with NumPy.
"""

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

WIDTH, HEIGHT = 640, 360
MAP = 512                  # heightmap cells per side (tileable)
CELL_M = 2.0               # metres per cell, so the terrain tile is 1024 m wide
DEPTHS = 240               # depth samples per ray
FAR_M = 900.0
FOCAL = WIDTH * 0.75
CAMERA_TILT_DEG = -8.0     # camera is mounted looking slightly down
HOME_LAT, HOME_LON = 48.8566, 2.3522
SUN_ELEVATION_DEG = 22.0   # low sun for long shadows; light comes from +x (east)

SKY_TOP = np.array([58, 112, 176], dtype=np.float32)
SKY_HORIZON = np.array([196, 214, 224], dtype=np.float32)

MODE_LABELS = ["STANDBY", "ARMED", "TAKEOFF", "SURVEY", "RETURN TO HOME", "LANDING", "CHARGING", "EMERGENCY"]


@dataclass
class Pose:
    north_m: float = 0.0
    east_m: float = 0.0
    altitude_m: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    mode: str = "STANDBY"
    failsafe: bool = False


# --------------------------------------------------------------------- terrain
def build_terrain(seed: int = 7) -> tuple[np.ndarray, np.ndarray]:
    """Return (height in metres, RGB colour with baked lighting), both MAP x MAP."""
    rng = np.random.default_rng(seed)
    freq = np.fft.fftfreq(MAP)
    fx, fy = np.meshgrid(freq, freq)
    radius = np.sqrt(fx * fx + fy * fy)
    radius[0, 0] = 1.0
    # 1/f^2 noise filtered in the frequency domain is smooth, natural and tileable.
    spectrum = (rng.normal(size=(MAP, MAP)) + 1j * rng.normal(size=(MAP, MAP))) / radius ** 1.9
    spectrum[radius > 0.2] = 0
    height = np.real(np.fft.ifft2(spectrum))
    height = (height - height.min()) / (height.max() - height.min())
    height = height ** 1.5 * 62.0                         # ridges and bumps up to ~60 m

    # Flatten a landing area around home (cell 0,0 wraps to all four corners).
    idx = np.minimum(np.arange(MAP), MAP - np.arange(MAP)) * CELL_M
    dist = np.sqrt(idx[None, :] ** 2 + idx[:, None] ** 2)
    flat = np.clip((dist - 40.0) / 150.0, 0.0, 1.0)  # the survey loop flies a valley
    flat = flat * flat * (3 - 2 * flat)
    height = height * flat + 0.3

    # Hillshade from the surface normal.
    gy, gx = np.gradient(height, CELL_M)
    sun = np.array([math.cos(math.radians(SUN_ELEVATION_DEG)), 0.0, math.sin(math.radians(SUN_ELEVATION_DEG))])
    normal = np.stack([-gx, -gy, np.ones_like(height)])
    normal /= np.linalg.norm(normal, axis=0)
    shade = np.clip(np.tensordot(sun, normal, axes=1), 0.0, 1.0)

    # Cast shadows: with the sun along +x, a cell is shadowed when some cell further
    # east rises above the sun ray, i.e. max_{x'>x}(h(x') - k x') > h(x) - k x.
    k = math.tan(math.radians(SUN_ELEVATION_DEG)) * CELL_M
    x = np.arange(MAP * 2, dtype=np.float32)
    tiled = np.concatenate([height, height], axis=1)        # wrap so the tile stays seamless
    g = tiled - k * x
    ahead = np.maximum.accumulate(g[:, ::-1], axis=1)[:, ::-1]
    ahead = np.concatenate([ahead[:, 1:], np.full((MAP, 1), -1e9)], axis=1)
    shadow = (ahead > g + 0.2)[:, :MAP]

    # Colour by height and slope: meadow, dry grass, rock.
    t = np.clip(height / 60.0, 0, 1)[..., None]
    slope = np.clip(np.hypot(gx, gy) * 1.6, 0, 1)[..., None]
    meadow = np.array([62, 104, 52], dtype=np.float32)
    dry = np.array([128, 132, 76], dtype=np.float32)
    rock = np.array([120, 110, 98], dtype=np.float32)
    base = meadow * (1 - t) + dry * t
    base = base * (1 - slope) + rock * slope
    texture = 0.92 + 0.16 * rng.random((MAP, MAP, 1), dtype=np.float32)   # grain so close ground reads
    light = (0.22 + 0.95 * shade[..., None]) * np.where(shadow[..., None], 0.38, 1.0)
    color = base * light * texture

    # Landing pad: a light disc with a dark ring at home.
    pad = dist < 9.0
    color[pad] = [170, 170, 162]
    color[(dist > 7.0) & (dist < 8.5)] = [60, 62, 64]
    return height.astype(np.float32), np.clip(color, 0, 255).astype(np.float32)


class Camera:
    def __init__(self) -> None:
        self.height, self.color = build_terrain()
        self.depth = np.geomspace(1.5, FAR_M, DEPTHS).astype(np.float32)
        self.fog = (1.0 - np.exp(-self.depth / 520.0)).astype(np.float32)[None, :, None]
        # Render with a margin on every side so rolling the image never shows corners.
        self.render_w, self.render_h = int(WIDTH * 1.2), int(HEIGHT * 1.3)
        self.lateral = ((np.arange(self.render_w) - self.render_w / 2) / FOCAL).astype(np.float32)
        self.rows = np.arange(self.render_h, dtype=np.float32)

    def render(self, pose: Pose) -> Image.Image:
        yaw = math.radians(pose.yaw_deg)
        fwd = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float32)     # north, east
        right = np.array([-math.sin(yaw), math.cos(yaw)], dtype=np.float32)
        cam_h = max(1.6, pose.altitude_m + float(self.sample_height(pose.north_m, pose.east_m)) + 1.2)

        # Sample points for every (column, depth): forward distance d, lateral d * tan.
        d = self.depth[None, :]
        lat = self.lateral[:, None] * d
        north = pose.north_m + fwd[0] * d + right[0] * lat
        east = pose.east_m + fwd[1] * d + right[1] * lat
        fx, fy = east / CELL_M, north / CELL_M
        x0, y0 = np.floor(fx).astype(np.int64), np.floor(fy).astype(np.int64)
        ax, ay = (fx - x0)[..., None], (fy - y0)[..., None]
        x0, y0 = x0 % MAP, y0 % MAP
        x1, y1 = (x0 + 1) % MAP, (y0 + 1) % MAP
        h = self.height[y0, x0]
        col = ((self.color[y0, x0] * (1 - ax) + self.color[y0, x1] * ax) * (1 - ay)
               + (self.color[y1, x0] * (1 - ax) + self.color[y1, x1] * ax) * ay)

        # Screen row of each sample; tilt (mount angle plus pitch) shears the horizon.
        # Looking down (negative tilt) raises the horizon on screen.
        tilt = math.radians(CAMERA_TILT_DEG + pose.pitch_deg)
        horizon = self.render_h / 2 + math.tan(tilt) * FOCAL
        ys = horizon + (cam_h - h) / d * FOCAL
        # Front-to-back: a sample is visible where it rises above everything nearer.
        top = np.minimum.accumulate(ys, axis=1)

        col = col * (1 - self.fog) + SKY_HORIZON * self.fog
        rows = self.rows
        # Sky gradient follows the horizon so it stays consistent while pitching.
        s = np.clip((rows - (horizon - self.render_h * 0.9)) / (self.render_h * 0.9), 0, 1)[:, None]
        sky = SKY_TOP * (1 - s) + SKY_HORIZON * s
        frame = np.empty((self.render_h, self.render_w, 3), dtype=np.float32)
        for c in range(self.render_w):
            # index of the first depth whose running top is at or above the row
            k = np.searchsorted(-top[c], -rows, side="left")
            visible = k < DEPTHS
            frame[:, c] = np.where(visible[:, None], col[c, np.minimum(k, DEPTHS - 1)], sky)
        if pose.failsafe:
            frame = frame * np.array([1.08, 0.94, 0.9], dtype=np.float32)

        image = Image.fromarray(np.clip(frame, 0, 255).astype(np.uint8))
        if abs(pose.roll_deg) > 0.05:
            image = image.rotate(-pose.roll_deg, resample=Image.Resampling.BILINEAR)
        left, upper = (self.render_w - WIDTH) // 2, (self.render_h - HEIGHT) // 2
        image = image.crop((left, upper, left + WIDTH, upper + HEIGHT))
        self.draw_hud(image, pose)
        return image

    def sample_height(self, north_m: float, east_m: float) -> float:
        return self.height[int(north_m // CELL_M) % MAP, int(east_m // CELL_M) % MAP]

    @staticmethod
    def draw_hud(image: Image.Image, pose: Pose) -> None:
        draw = ImageDraw.Draw(image)
        cx, cy = WIDTH // 2, HEIGHT // 2
        draw.line([(cx - 14, cy), (cx - 5, cy)], fill=(235, 240, 245), width=1)
        draw.line([(cx + 5, cy), (cx + 14, cy)], fill=(235, 240, 245), width=1)
        draw.line([(cx, cy - 5), (cx, cy + 5)], fill=(235, 240, 245), width=1)
        text = f"{pose.mode}   ALT {pose.altitude_m:5.1f} m   HDG {pose.yaw_deg % 360:03.0f}"
        draw.rectangle((8, HEIGHT - 26, 16 + 6 * len(text), HEIGHT - 8), fill=(10, 14, 20))
        draw.text((12, HEIGHT - 23), text, fill=(235, 240, 245))


# ------------------------------------------------------------------ telemetry
POSE_PARAMETERS = ["/drone/Navigation.latitude", "/drone/Navigation.longitude", "/drone/Navigation.altitude",
                   "/drone/Attitude.roll", "/drone/Attitude.pitch", "/drone/Attitude.yaw",
                   "/drone/FlightMode", "/drone/FailsafeActive"]


def read_pose(processor, previous: Pose) -> Pose:
    try:
        values = processor.get_parameter_values(POSE_PARAMETERS)
    except Exception as exc:                              # keep the feed alive on hiccups
        print(f"telemetry read failed: {exc}", flush=True)
        return previous
    v = {name: (pv.eng_value if pv is not None else None) for name, pv in zip(POSE_PARAMETERS, values)}

    def num(name: str, default: float) -> float:
        value = v.get(name)
        return float(value) if isinstance(value, (int, float)) else default

    lat = num("/drone/Navigation.latitude", HOME_LAT)
    lon = num("/drone/Navigation.longitude", HOME_LON)
    mode = v.get("/drone/FlightMode")
    return Pose(north_m=(lat - HOME_LAT) * 111_320.0,
                east_m=(lon - HOME_LON) * 111_320.0 * math.cos(math.radians(HOME_LAT)),
                altitude_m=num("/drone/Navigation.altitude", 0.0),
                roll_deg=num("/drone/Attitude.roll", 0.0),
                pitch_deg=num("/drone/Attitude.pitch", 0.0),
                yaw_deg=num("/drone/Attitude.yaw", 0.0),
                mode=str(mode).replace("_", " ") if mode else previous.mode,
                failsafe=bool(v.get("/drone/FailsafeActive")))


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
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--yamcs-host", default=os.getenv("YAMCS_HOST", "localhost:8090"))
    parser.add_argument("--public-yamcs-url", default=os.getenv("PUBLIC_YAMCS_URL", "http://localhost:8090"))
    parser.add_argument("--instance", default=os.getenv("YAMCS_INSTANCE", "demo"))
    parser.add_argument("--processor", default=os.getenv("YAMCS_PROCESSOR", "realtime"))
    parser.add_argument("--image-dir", type=Path, default=Path(os.getenv("IMAGE_DIR", "/tmp/images")))
    parser.add_argument("--interval", type=float, default=float(os.getenv("IMAGE_INTERVAL", "0.25")),
                        help="seconds between frames (0.25 = 4 frames per second)")
    parser.add_argument("--retain", type=int, default=int(os.getenv("IMAGE_RETAIN", "200")))
    parser.add_argument("--username", default=os.getenv("YAMCS_USERNAME"))
    parser.add_argument("--password", default=os.getenv("YAMCS_PASSWORD"))
    parser.add_argument("--render-only", type=Path, help="render one sample frame without connecting to Yamcs")
    parser.add_argument("--once", action="store_true", help="generate one frame and exit")
    args = parser.parse_args()

    args.image_dir.mkdir(parents=True, exist_ok=True)
    camera = Camera()
    if args.render_only:
        sample = Pose(north_m=140.0, east_m=-60.0, altitude_m=40.0, roll_deg=2.6, pitch_deg=-6.5,
                      yaw_deg=110.0, mode="SURVEY")
        camera.render(sample).save(args.render_only, quality=85)
        print(f"rendered {args.render_only}")
        return

    processor = connect_processor(args.yamcs_host, args.instance, args.processor, args.username, args.password)
    pose = Pose()
    frame = 1
    while True:
        started = time.monotonic()
        pose = read_pose(processor, pose)
        image_name = f"drone_camera_{frame:05d}.jpg"
        camera.render(pose).save(args.image_dir / image_name, quality=82)

        storage_url = f"/storage/buckets/images/objects/{image_name}"
        public_url = f"{args.public_yamcs_url.rstrip('/')}/api{storage_url}"
        processor.set_parameter_values({
            "/drone/CameraImageNumber": frame,
            "/drone/CameraImageStorageUrl": storage_url,
            "/drone/CameraImageUrl": public_url,
            "/drone/CameraView": "forward-terrain",
        })

        if args.retain > 0 and frame > args.retain:
            (args.image_dir / f"drone_camera_{frame - args.retain:05d}.jpg").unlink(missing_ok=True)
        if frame % 40 == 0:
            print(f"frame {frame}: {public_url} ({(time.monotonic() - started) * 1000:.0f} ms)", flush=True)
        if args.once:
            return
        frame += 1
        time.sleep(max(0.0, args.interval - (time.monotonic() - started)))


if __name__ == "__main__":
    main()
