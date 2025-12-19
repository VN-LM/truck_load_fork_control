#!/usr/bin/env python3

import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Vec2:
    x: float
    z: float


def rot(theta: float, x: float, z: float) -> Vec2:
    c = math.cos(theta)
    s = math.sin(theta)
    return Vec2(c * x - s * z, s * x + c * z)


def env_floor_z_at_x(cfg: dict, x_cpp: float) -> float:
    env = cfg["environment"]
    door = float(env["container"]["door_x"])
    floor_z = float(env["container"]["floor_z"])
    container_len = float(env["container"]["length"])
    ramp_l = float(env["ramp"]["length"])
    slope_deg = float(env["ramp"]["slope_deg"])

    h = math.tan(math.radians(slope_deg)) * ramp_l
    ground_z = floor_z - h
    ramp_start_x = door - ramp_l

    # C++ environment convention:
    # - container spans [door, door + len]
    # - ramp is on left side: [door - rampL, door]
    if door <= x_cpp <= door + container_len:
        return floor_z
    if x_cpp <= ramp_start_x:
        return ground_z

    # on ramp: linear interpolation from ground_z at ramp_start_x up to floor_z at door
    t = (x_cpp - ramp_start_x) / (door - ramp_start_x)
    return ground_z + t * (floor_z - ground_z)


def predicted_corners_cpp(cfg: dict, row: dict) -> dict:
    s = float(row["s"])
    pitch = float(row["pitch"])
    tilt = float(row["tilt"])
    lift = float(row["lift"])

    theta = pitch + tilt

    mast_pivot_h = float(cfg["vehicle"]["mast"]["pivot_height"])
    cargo_l = float(cfg["cargo"]["length"])
    cargo_h = float(cfg["cargo"]["height"])
    mo = cfg.get("cargo", {}).get("mount_offset", {})
    mount_x = float(mo.get("x", 0.0))
    mount_z = float(mo.get("z", 0.0))

    floor_at_mast = env_floor_z_at_x(cfg, s)
    mast_base = Vec2(s, floor_at_mast + mast_pivot_h)

    lift_vec = rot(theta, 0.0, lift)
    pivot = Vec2(mast_base.x + lift_vec.x, mast_base.z + lift_vec.z)

    rb_off = rot(theta, mount_x, mount_z)
    rt_off = rot(theta, mount_x, mount_z + cargo_h)
    fb_off = rot(theta, mount_x + cargo_l, mount_z)
    ft_off = rot(theta, mount_x + cargo_l, mount_z + cargo_h)

    rb = Vec2(pivot.x + rb_off.x, pivot.z + rb_off.z)
    rt = Vec2(pivot.x + rt_off.x, pivot.z + rt_off.z)
    fb = Vec2(pivot.x + fb_off.x, pivot.z + fb_off.z)
    ft = Vec2(pivot.x + ft_off.x, pivot.z + ft_off.z)

    return {
        "rb": rb,
        "rt": rt,
        "fb": fb,
        "ft": ft,
    }


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]
    default_csv = repo_root / "tlf_log.csv"
    default_cfg = Path(__file__).resolve().parent / "model_config_default.json"

    csv_path = Path(sys.argv[1]) if len(sys.argv) >= 2 else default_csv
    cfg_path = Path(sys.argv[2]) if len(sys.argv) >= 3 else default_cfg

    cfg = json.loads(cfg_path.read_text(encoding="utf-8"))

    tol = float(cfg.get("viewer", {}).get("validation_tol", 1e-5))

    max_err = 0.0
    max_err_frame = None
    max_err_key = None

    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            pred = predicted_corners_cpp(cfg, row)

            # Compare to logged corners (C++ coords)
            for key in ["rb", "rt", "fb", "ft"]:
                px = pred[key].x
                pz = pred[key].z
                lx = float(row[f"{key}_x"])
                lz = float(row[f"{key}_z"])

                err = max(abs(px - lx), abs(pz - lz))
                if err > max_err:
                    max_err = err
                    max_err_frame = idx
                    max_err_key = key

    ok = max_err <= tol
    status = "OK" if ok else "FAIL"
    print(f"{status}: max corner error = {max_err:.6g} (tol={tol})")
    if max_err_frame is not None:
        print(f"worst frame index: {max_err_frame}, corner: {max_err_key}")

    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
