#!/usr/bin/env python3
"""Would each source have tripped the reflex? Replay the collision-monitor
geometry offline over an extracted window.

Reproduces what `segments_to_pointcloud_reflex` + nav2 Collision Monitor do on
the boat (sea_surface_segmentation/segments_projection.hpp, bizzyboat
perception_launch.py + nav2_overlay.yaml), for each of the three sources:

  1. obstacle pixel  = argmax class is obstacle (R > G and R > B)
  2. confidence gate = P(obstacle) >= obstacle_prob_min (reflex: 0.60 since
                       2026-06-09, unset = 0.0 before)
  3. ray            = pinhole model from the recorded 128x96 camera_info
                       (rational-polynomial distortion, undistorted with OpenCV
                       as image_geometry::projectPixelTo3dRay does)
  4. rotate ray into `bizzy/base_link_level` using the bag's own /tf and
     /tf_static (base_link_north_up -> base_link_level and -> base_link at the
     frame stamp; base_link -> oak_<cam> -> oak_<cam>_optical static)
  5. intersect the plane z = projection_plane_z (0.0 as deployed); drop rays
     that miss or point away
  6. count points inside the CollisionStop box ([0,5] x [-2,2] m, min_points 5)
     and the CollisionSlowdown box ([0,20] x [-3,3] m, min_points 4)

Reports per source: share of frames that would have triggered stop / slowdown,
number of distinct trigger episodes (runs of consecutive triggering frames),
and per-frame counts to timeseries_reflex.csv. Single camera only: the boat
merges all four cameras into one cloud, so a per-camera count is a lower bound
on what the boat would have seen.

Writes <window>/reflex.csv and <window>/timeseries_reflex.csv.
Part of rolker/unh_marine_perception#49 (evidence for #42).
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from scipy.spatial.transform import Rotation as R

STOP = (0.0, 5.0, -2.0, 2.0, 5)       # xmin, xmax, ymin, ymax, min_points
SLOW = (0.0, 20.0, -3.0, 3.0, 4)


def find_mcap(path: str) -> str:
    p = Path(path)
    if p.is_file():
        return str(p)
    mcaps = sorted(p.glob("*.mcap"))
    if len(mcaps) != 1:
        raise ValueError(f"expected exactly one .mcap in {path}")
    return str(mcaps[0])


def tf_to_mat(t) -> np.ndarray:
    m = np.eye(4)
    q = t.transform.rotation
    m[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    v = t.transform.translation
    m[:3, 3] = [v.x, v.y, v.z]
    return m


def stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def load_geometry(mcap_path: str, ns: str, cam: str, t0: int, t1: int):
    """camera_info (first in window) + static chain base_link->optical +
    time series of T(north_up->level) and T(north_up->base_link)."""
    ci = None
    static: dict[tuple[str, str], np.ndarray] = {}
    lvl: list[tuple[int, np.ndarray]] = []
    base: list[tuple[int, np.ndarray]] = []
    ci_topic = f"{ns}/sensors/cameras/{cam}/segmentation/camera_info"
    pad = 2_000_000_000
    with open(mcap_path, "rb") as fp:
        reader = make_reader(fp, decoder_factories=[DecoderFactory()])
        for _s, ch, _m, msg in reader.iter_decoded_messages(topics=["/tf_static"]):
            for t in msg.transforms:
                static[(t.header.frame_id, t.child_frame_id)] = tf_to_mat(t)
        for _s, ch, _m, msg in reader.iter_decoded_messages(topics=["/tf", ci_topic], start_time=t0 - pad, end_time=t1 + pad):
            if ch.topic == ci_topic:
                if ci is None:
                    ci = msg
                continue
            for t in msg.transforms:
                if t.header.frame_id == f"{ns[1:]}/base_link_north_up":
                    if t.child_frame_id == f"{ns[1:]}/base_link_level":
                        lvl.append((stamp_ns(t.header), tf_to_mat(t)))
                    elif t.child_frame_id == f"{ns[1:]}/base_link":
                        base.append((stamp_ns(t.header), tf_to_mat(t)))
    if ci is None:
        raise RuntimeError("no camera_info in window")
    b = ns[1:]
    chain = [(f"{b}/base_link", f"{b}/{cam}"), (f"{b}/{cam}", f"{b}/{cam}_optical")]
    T_base_opt = np.eye(4)
    for key in chain:
        if key not in static:
            raise RuntimeError(f"missing static transform {key}")
        T_base_opt = T_base_opt @ static[key]
    if not lvl or not base:
        raise RuntimeError("no dynamic base_link_level / base_link transforms in window")
    lvl.sort(key=lambda x: x[0])
    base.sort(key=lambda x: x[0])
    return ci, T_base_opt, lvl, base


def nearest(series: list[tuple[int, np.ndarray]], stamps: np.ndarray, t: int) -> tuple[np.ndarray, float]:
    j = int(np.searchsorted(stamps, t))
    cands = [k for k in (j - 1, j) if 0 <= k < len(series)]
    k = min(cands, key=lambda k: abs(int(stamps[k]) - t))
    return series[k][1], abs(int(stamps[k]) - t) / 1e9


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--window", required=True)
    ap.add_argument("--bag", required=True, help="the camera bag the window came from (for tf + camera_info)")
    ap.add_argument("--camera", required=True)
    ap.add_argument("--namespace", default="/bizzy")
    ap.add_argument("--obstacle-prob-min", type=float, default=0.60)
    ap.add_argument("--plane-z", type=float, default=0.0)
    ap.add_argument("--tf-tol-s", type=float, default=1.0, help="max age of the nearest tf sample")
    args = ap.parse_args()
    win = Path(args.window)

    fr = np.load(win / "frames.npz")
    stamps = fr["stamp_ns"]
    n = len(stamps)
    mk = np.load(win / "masks.npz")
    a = mk["masks"].astype(np.float32) / 255.0
    a[mk["stamp_ns"] < 0] = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    sources = {"A_recorded": a}
    p = win / "probs_ewasr_offline.npz"
    if p.exists():
        sources["B_ewasr_offline"] = np.load(p)["probs"].astype(np.float32)
    for p in sorted(win.glob("probs_wasrt_h*.npz")):
        sources["C_" + p.stem.replace("probs_", "")] = np.load(p)["probs"].astype(np.float32)
    for k in sources:
        sources[k] = sources[k][:n]

    ci, T_base_opt, lvl, base = load_geometry(find_mcap(args.bag), args.namespace, args.camera, int(stamps[0]), int(stamps[-1]))
    lvl_st = np.array([s for s, _ in lvl], dtype=np.int64)
    base_st = np.array([s for s, _ in base], dtype=np.int64)
    h, w = ci.height, ci.width
    K = np.array(ci.k, dtype=np.float64).reshape(3, 3)
    D = np.array(ci.d, dtype=np.float64)
    # Undistorted unit-plane rays for every pixel centre, as projectPixelTo3dRay:
    # rectify the pixel through K/D, then (x, y, 1) in the optical frame.
    uv = np.array([[c, r] for r in range(h) for c in range(w)], dtype=np.float64).reshape(-1, 1, 2)
    und = cv2.undistortPoints(uv, K, D).reshape(-1, 2)  # normalised coordinates
    rays_opt = np.concatenate([und, np.ones((und.shape[0], 1))], axis=1).reshape(h, w, 3)

    def project(mask_obst: np.ndarray, T_lvl_opt: np.ndarray) -> np.ndarray:
        rays = rays_opt[mask_obst] @ T_lvl_opt[:3, :3].T
        o = T_lvl_opt[:3, 3]
        rz = rays[:, 2]
        ok = rz != 0.0
        u = np.full(len(rays), np.nan)
        u[ok] = (args.plane_z - o[2]) / rz[ok]
        ok &= np.isfinite(u) & (u > 0)
        pts = o[None, :2] + u[ok, None] * rays[ok, :2]
        return pts

    def in_box(pts: np.ndarray, box) -> int:
        if len(pts) == 0:
            return 0
        x, y = pts[:, 0], pts[:, 1]
        return int(((x >= box[0]) & (x <= box[1]) & (y >= box[2]) & (y <= box[3])).sum())

    counts = {k: np.zeros((n, 2), dtype=int) for k in sources}  # stop, slow
    nearest_x = {k: np.full(n, np.nan) for k in sources}
    tf_missing = 0
    for i in range(n):
        t = int(stamps[i])
        T_nu_lvl, age1 = nearest(lvl, lvl_st, t)
        T_nu_base, age2 = nearest(base, base_st, t)
        if max(age1, age2) > args.tf_tol_s:
            tf_missing += 1
            continue
        T_lvl_opt = np.linalg.inv(T_nu_lvl) @ T_nu_base @ T_base_opt
        for k, probs in sources.items():
            pr = probs[i]
            m = (pr.argmax(-1) == 0) & (pr[..., 0] >= args.obstacle_prob_min)
            pts = project(m, T_lvl_opt)
            counts[k][i] = (in_box(pts, STOP), in_box(pts, SLOW))
            if len(pts):
                fwd = pts[(pts[:, 0] > 0) & (np.abs(pts[:, 1]) <= 3.0)]
                if len(fwd):
                    nearest_x[k][i] = float(fwd[:, 0].min())

    def episodes(flags: np.ndarray) -> int:
        f = flags.astype(int)
        return int(((f[1:] == 1) & (f[:-1] == 0)).sum() + (1 if len(f) and f[0] else 0))

    rows = []
    for k in sources:
        stop = counts[k][:, 0] >= STOP[4]
        slow = counts[k][:, 1] >= SLOW[4]
        rows.append({
            "source": k, "frames": n, "obstacle_prob_min": args.obstacle_prob_min,
            "stop_frames": int(stop.sum()), "stop_frac": float(stop.mean()), "stop_episodes": episodes(stop),
            "slow_frames": int(slow.sum()), "slow_frac": float(slow.mean()), "slow_episodes": episodes(slow),
            "median_nearest_x_m": float(np.nanmedian(nearest_x[k])) if np.isfinite(nearest_x[k]).any() else float("nan"),
        })
    with open(win / "reflex.csv", "w", newline="") as fp:
        wri = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        wri.writeheader()
        wri.writerows(rows)
    with open(win / "timeseries_reflex.csv", "w", newline="") as fp:
        wri = csv.writer(fp)
        wri.writerow(["idx", "stamp_ns"] + [f"{k}_{b}" for k in sources for b in ("stop_pts", "slow_pts")])
        for i in range(n):
            wri.writerow([i, int(stamps[i])] + [int(v) for k in sources for v in counts[k][i]])

    cam_h = (np.linalg.inv(lvl[0][1]) @ base[0][1] @ T_base_opt)[2, 3]
    print(f"{win.parent.name}/{win.name}: {n} frames, camera {cam_h:.2f} m above plane z={args.plane_z}, "
          f"obstacle_prob_min={args.obstacle_prob_min}, tf missing {tf_missing}")
    print(f"{'source':>18} {'stop frames':>12} {'stop %':>7} {'episodes':>9} {'slow frames':>12} {'slow %':>7} {'episodes':>9}")
    for r in rows:
        print(f"{r['source']:>18} {r['stop_frames']:>12} {100 * r['stop_frac']:>7.1f} {r['stop_episodes']:>9} "
              f"{r['slow_frames']:>12} {100 * r['slow_frac']:>7.1f} {r['slow_episodes']:>9}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
