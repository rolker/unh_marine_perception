#!/usr/bin/env python3
"""Contact sheet image for one window: the K frames with the most interior-water
obstacle pixels in source A (recorded eWaSR), plus evenly spaced context
frames, each as RGB | A | B | C with obstacle tinted red. As JPEG it is small
enough to commit under results/. Part of rolker/unh_marine_perception#49."""

from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_overlay import tint  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--window", required=True)
    ap.add_argument("--out", required=True, help="output image (.jpg recommended; .png also works)")
    ap.add_argument("--top", type=int, default=4, help="frames with most A obstacle px (water band)")
    ap.add_argument("--context", type=int, default=2, help="evenly spaced extra frames")
    ap.add_argument("--scale", type=float, default=0.5)
    args = ap.parse_args()
    win = Path(args.window)

    fr = np.load(win / "frames.npz")
    frames, stamps = fr["frames"], fr["stamp_ns"]
    mk = np.load(win / "masks.npz")
    a = mk["masks"].astype(np.float32) / 255.0
    a[mk["stamp_ns"] < 0] = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    sources = [("A rec", a)]
    p = win / "probs_ewasr_offline.npz"
    if p.exists():
        sources.append(("B off", np.load(p)["probs"].astype(np.float32)))
    for p in sorted(win.glob("probs_wasrt_h*.npz")):
        sources.append((f"C {p.stem.split('_')[-1]}", np.load(p)["probs"].astype(np.float32)))
    n = min([len(frames)] + [len(s[1]) for s in sources])

    # water-band obstacle count for A from timeseries.csv if present, else whole-frame
    counts = None
    ts = win / "timeseries.csv"
    if ts.exists():
        with open(ts) as fp:
            r = list(csv.DictReader(fp))
        col = next((k for k in r[0] if k.startswith("interior_px[A")), None)
        if col:
            counts = np.array([int(x[col]) for x in r])[:n]
    if counts is None:
        counts = (a[:n].argmax(-1) == 0).sum(axis=(1, 2))
    top = list(np.argsort(-counts)[: args.top])
    ctx = list(np.linspace(0, n - 1, args.context + 2, dtype=int)[1:-1]) if args.context else []
    idx = sorted(set(int(i) for i in top + ctx))

    font = cv2.FONT_HERSHEY_SIMPLEX
    rows = []
    for i in idx:
        f = frames[i]
        panels = [f.copy()] + [tint(f, s[1][i]) for s in sources]
        labels = [datetime.fromtimestamp(int(stamps[i]) / 1e9).strftime("%H:%M:%S") + f" #{i}"] + [
            f"{s[0]} {100 * float((s[1][i].argmax(-1) == 0).mean()):.1f}%" for s in sources]
        row = np.concatenate(panels, axis=1)
        for k, lab in enumerate(labels):
            cv2.putText(row, lab, (k * 512 + 8, 28), font, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(row, lab, (k * 512 + 8, 28), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        rows.append(row)
    sheet = np.concatenate(rows, axis=0)
    if args.scale != 1.0:
        sheet = cv2.resize(sheet, (int(sheet.shape[1] * args.scale), int(sheet.shape[0] * args.scale)), interpolation=cv2.INTER_AREA)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    # JPEG by default: these are photographs, and the repo's pre-commit
    # large-file hook caps committed files at 500 KB (a PNG sheet is ~700 KB).
    params = [cv2.IMWRITE_JPEG_QUALITY, 88] if args.out.lower().endswith((".jpg", ".jpeg")) else [cv2.IMWRITE_PNG_COMPRESSION, 9]
    cv2.imwrite(args.out, cv2.cvtColor(sheet, cv2.COLOR_RGB2BGR), params)
    print(f"wrote {args.out} frames={idx} A_px={[int(counts[i]) for i in idx]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
