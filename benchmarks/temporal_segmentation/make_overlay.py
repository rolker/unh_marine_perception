#!/usr/bin/env python3
"""Side-by-side overlay video for eyeball review of one extracted window.

Panels (left to right): decoded RGB frame | A recorded on-camera eWaSR |
B eWaSR offline | C WaSR-T. Each mask panel is the frame tinted red where the
source's argmax class is obstacle and blue where sky; water is left untinted.
Frame index, local timestamp and per-panel obstacle fraction are burned in.

Output: <window>/overlay.mp4 (H.264, 5 fps, playback speed 1x unless --speed).
Part of rolker/unh_marine_perception#49 (evidence for #42).
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

PANEL_W, PANEL_H = 512, 384


def tint(frame_rgb: np.ndarray, probs: np.ndarray) -> np.ndarray:
    cls = cv2.resize(probs.argmax(axis=-1).astype(np.uint8), (PANEL_W, PANEL_H), interpolation=cv2.INTER_NEAREST)
    out = frame_rgb.astype(np.float32)
    red = np.array([255, 40, 40], np.float32)
    blue = np.array([60, 120, 255], np.float32)
    out[cls == 0] = 0.45 * out[cls == 0] + 0.55 * red
    out[cls == 2] = 0.8 * out[cls == 2] + 0.2 * blue
    return out.astype(np.uint8)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--window", required=True)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args()
    win = Path(args.window)

    fr = np.load(win / "frames.npz")
    frames, stamps = fr["frames"], fr["stamp_ns"]
    mk = np.load(win / "masks.npz")
    a = mk["masks"].astype(np.float32) / 255.0
    a[mk["stamp_ns"] < 0] = np.array([0.0, 1.0, 0.0], dtype=np.float32)  # no recorded mask -> neutral, not "all obstacle"
    sources = [("A recorded eWaSR", a)]
    p = win / "probs_ewasr_offline.npz"
    if p.exists():
        sources.append(("B eWaSR offline (H.265 in)", np.load(p)["probs"].astype(np.float32)))
    for p in sorted(win.glob("probs_wasrt_h*.npz")):
        sources.append((f"C WaSR-T {p.stem.split('_')[-1]}", np.load(p)["probs"].astype(np.float32)))
    n = min([len(frames)] + [len(s[1]) for s in sources])
    if args.max_frames:
        n = min(n, args.max_frames)

    cols = 1 + len(sources)
    out_path = win / "overlay.mp4"
    vw = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"avc1"), 5.0 * args.speed, (PANEL_W * cols, PANEL_H))
    if not vw.isOpened():
        vw = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), 5.0 * args.speed, (PANEL_W * cols, PANEL_H))
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i in range(n):
        f = frames[i]
        panels = [f.copy()]
        labels = [datetime.fromtimestamp(int(stamps[i]) / 1e9).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + f"  #{i}"]
        for name, probs in sources:
            panels.append(tint(f, probs[i]))
            labels.append(f"{name}  obst {100 * float((probs[i].argmax(-1) == 0).mean()):.1f}%")
        row = np.concatenate(panels, axis=1)
        for k, lab in enumerate(labels):
            cv2.putText(row, lab, (k * PANEL_W + 8, 22), font, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(row, lab, (k * PANEL_W + 8, 22), font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        vw.write(cv2.cvtColor(row, cv2.COLOR_RGB2BGR))
    vw.release()
    print(f"wrote {out_path} ({n} frames, {cols} panels)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
