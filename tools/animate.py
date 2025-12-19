#!/usr/bin/env python3

import argparse
import csv
import math
from dataclasses import dataclass
from typing import List

try:
    import matplotlib.pyplot as plt
    from matplotlib import animation
except ModuleNotFoundError as e:
    missing = str(e)
    raise SystemExit(
        "Missing Python dependency (e.g., matplotlib/Pillow).\n"
        "Install with:\n"
        "  python3 -m pip install -r tools/requirements.txt\n\n"
        f"Original error: {missing}"
    )


@dataclass
class Sample:
    t: float
    s: float
    pitch: float
    lift: float
    tilt: float
    ceiling_z: float
    floor_z: float
    corners: List[tuple]
    clear_top: float
    clear_bottom: float
    safety_level: int
    terrain_state: int


def load_csv(path: str) -> List[Sample]:
    out: List[Sample] = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            corners = [
                (float(r["rb_x"]), float(r["rb_z"])),
                (float(r["rt_x"]), float(r["rt_z"])),
                (float(r["fb_x"]), float(r["fb_z"])),
                (float(r["ft_x"]), float(r["ft_z"])),
            ]
            out.append(
                Sample(
                    t=float(r["time"]),
                    s=float(r["s"]),
                    pitch=float(r["pitch"]),
                    lift=float(r["lift"]),
                    tilt=float(r["tilt"]),
                    ceiling_z=float(r["ceiling_z"]),
                    floor_z=float(r["floor_z"]),
                    corners=corners,
                    clear_top=float(r["clearance_top"]),
                    clear_bottom=float(r["clearance_bottom"]),
                    safety_level=int(r["safety_level"]),
                    terrain_state=int(r["terrain_state"]),
                )
            )
    return out


def ramp_floor_z(x: float, ramp_deg: float = 4.0) -> float:
    a = math.tan(math.radians(ramp_deg))
    return a * x if x < 0.0 else 0.0


def safety_color(level: int):
    if level == 0:
        return "#4fc37a"
    if level == 1:
        return "#f0c850"
    if level == 2:
        return "#f05050"
    return "#a0a0dc"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--out", required=True, help="Output .gif or .mp4")
    ap.add_argument("--fps", type=int, default=30)
    args = ap.parse_args()

    samples = load_csv(args.log)
    if not samples:
        raise SystemExit("empty log")

    x_min, x_max = -2.0, 2.2
    z_min, z_max = -0.8, 3.0

    fig = plt.figure(figsize=(10, 6))
    gs = fig.add_gridspec(2, 1, height_ratios=[3, 1])

    ax = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[1, 0])

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(z_min, z_max)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)")
    ax.set_title("Truck load fork control: 2D replay")

    ax2.set_xlim(samples[0].t, samples[-1].t)
    ax2.set_ylim(min(min(s.clear_top, s.clear_bottom) for s in samples) - 0.1, 0.6)
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("clearance (m)")

    floor_line, = ax.plot([], [], lw=2, color="#8888aa")
    ceiling_line, = ax.plot([], [], lw=1, color="#777788")
    door_line, = ax.plot([0, 0], [0, samples[0].ceiling_z], lw=2, color="#ccccdd")

    rack_poly, = ax.plot([], [], lw=3)
    pivot_pt, = ax.plot([], [], marker="o", color="#dddddd")

    top_curve, = ax2.plot([], [], lw=1.5, color="#4fc37a", label="clear_top")
    bottom_curve, = ax2.plot([], [], lw=1.5, color="#f0c850", label="clear_bottom")
    marker_t = ax2.axvline(samples[0].t, color="#aaaaaa", lw=1)
    ax2.legend(loc="upper right")

    times = [s.t for s in samples]
    tops = [s.clear_top for s in samples]
    bottoms = [s.clear_bottom for s in samples]

    top_curve.set_data(times, tops)
    bottom_curve.set_data(times, bottoms)

    xs = [x_min + (x_max - x_min) * i / 80 for i in range(81)]
    floor = [ramp_floor_z(x) for x in xs]
    floor_line.set_data(xs, floor)

    def update(i: int):
        s = samples[i]
        ceiling_line.set_data([x_min, x_max], [s.ceiling_z, s.ceiling_z])
        door_line.set_data([0, 0], [0, s.ceiling_z])

        rb, rt, fb, ft = s.corners
        poly = [rb, fb, ft, rt, rb]
        rack_poly.set_data([p[0] for p in poly], [p[1] for p in poly])
        rack_poly.set_color(safety_color(s.safety_level))

        pivot_pt.set_data([s.s], [s.lift])

        marker_t.set_xdata([s.t, s.t])
        ax.set_title(
            f"t={s.t:.2f}s  s={s.s:.2f}m  clear_top={s.clear_top:.3f}m  clear_bottom={s.clear_bottom:.3f}m"
        )
        return rack_poly, pivot_pt, ceiling_line, door_line, marker_t

    ani = animation.FuncAnimation(fig, update, frames=len(samples), interval=1000 / args.fps, blit=False)

    if args.out.lower().endswith(".gif"):
        ani.save(args.out, writer=animation.PillowWriter(fps=args.fps))
    else:
        # mp4 requires ffmpeg installed
        ani.save(args.out, writer="ffmpeg", fps=args.fps)

    print(f"Wrote animation: {args.out}")


if __name__ == "__main__":
    main()
