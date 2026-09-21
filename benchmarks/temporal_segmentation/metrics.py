#!/usr/bin/env python3
"""Compare obstacle masks from the three sources over an extracted window.

Sources (all at 128x96, [obstacle, water, sky] probabilities):
  A  recorded     on-camera eWaSR on the uncompressed preview   (masks.npz)
  B  ewasr_offline eWaSR ONNX on the decoded H.265 frames        (probs_ewasr_offline.npz)
  C  wasrt_h<H>   WaSR-T temporal model on the same frames       (probs_wasrt_h*.npz)

A-vs-B is the compression effect, B-vs-C is the model effect on equal input.

No ground truth exists for this footage, so the numbers are proxies for the
false-positive behaviour that hurts the boat:

  obst_frac         mean fraction of pixels classed obstacle per frame, under
                    the deployed rule (argmax == obstacle) and under the
                    obstacle_prob_min thresholds the field used (0.7 / 0.8 /
                    0.95 on the reflex node).
  obst_frac_water   same, restricted to the per-frame "water band": rows below
                    that frame's horizon row (first row, top-down, where the
                    column-median sky probability from source A drops under
                    0.5) plus a 4-row margin. The boat rolls and pitches, so
                    the band is per frame, not per window. This is where a
                    false positive becomes a costmap mark; the shoreline sits
                    on the horizon and is mostly excluded by the margin.
  interior_px       mean obstacle pixels per frame (argmax rule) in connected
                    components that do NOT touch the horizon band, i.e.
                    detections floating in open water rather than the shore /
                    horizon strip. This is the glint / clutter signal proper;
                    obst_frac_water is dominated by the shoreline strip.
  interior_blobs    mean number of such components per frame.
  interior_frames   fraction of frames with at least one interior component.
  flicker           mean fraction of pixels whose argmax class changes between
                    consecutive frames (the frame-to-frame instability the
                    2026-05-22 log describes).
  blobs             mean number of 4-connected obstacle components per frame
                    in the water band (each blob is a potential reflex point).
  persist_3of4      fraction of obstacle pixels (water band) that are obstacle
                    in >= 3 of the last 4 frames -- what a persistence gate
                    would keep; higher means the detections are stable, lower
                    means glint-like flicker.

Writes <window>/metrics.csv (one row per source), <window>/timeseries.csv
(per frame: horizon row and interior obstacle pixel count per source, for
locating incidents) and prints a table.
Part of rolker/unh_marine_perception#49 (evidence for #42).
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

THRESHOLDS = (0.7, 0.8, 0.95)
HORIZON_MARGIN_ROWS = 4


def load_sources(win: Path) -> dict[str, np.ndarray]:
    src: dict[str, np.ndarray] = {}
    m = np.load(win / "masks.npz")
    valid = m["stamp_ns"] >= 0
    a = m["masks"].astype(np.float32) / 255.0
    # A frame with no recorded mask is stored all-zero; argmax of zeros is class
    # 0 = obstacle, which would count a data gap as a full-frame detection.
    # Substitute "all water" so gaps are neutral in every statistic below.
    a[~valid] = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    src["A_recorded"] = a
    p = win / "probs_ewasr_offline.npz"
    if p.exists():
        src["B_ewasr_offline"] = np.load(p)["probs"].astype(np.float32)
    for p in sorted(win.glob("probs_wasrt_h*.npz")):
        src["C_" + p.stem.replace("probs_", "")] = np.load(p)["probs"].astype(np.float32)
    n = min(len(v) for v in src.values())
    for k in src:
        src[k] = src[k][:n]
    src["_valid"] = valid[:n]
    return src


def horizon_rows(probs_a: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Per-frame first row (top-down) where the column-median sky prob < 0.5;
    frames with no recorded mask take the nearest valid frame's row."""
    n, h = probs_a.shape[:2]
    rows = np.full(n, -1, dtype=int)
    for i in np.flatnonzero(valid):
        sky = np.median(probs_a[i, :, :, 2], axis=1)
        below = np.flatnonzero(sky < 0.5)
        rows[i] = int(below[0]) if len(below) else h
    vi = np.flatnonzero(rows >= 0)
    if len(vi) == 0:
        return np.zeros(n, dtype=int)
    for i in np.flatnonzero(rows < 0):
        rows[i] = rows[vi[np.abs(vi - i).argmin()]]
    return rows


def water_band(rows: np.ndarray, h: int, w: int) -> np.ndarray:
    band = np.zeros((len(rows), h, w), dtype=bool)
    for i, r in enumerate(rows):
        band[i, min(h - 1, int(r) + HORIZON_MARGIN_ROWS):, :] = True
    return band


def obstacle_masks(probs: np.ndarray) -> dict[str, np.ndarray]:
    am = probs.argmax(axis=-1) == 0
    out = {"argmax": am}
    for t in THRESHOLDS:
        out[f"p>={t}"] = probs[..., 0] >= t
    return out


def interior_components(am: np.ndarray, hrows: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per frame: (obstacle pixels, component count) for 4-connected obstacle
    components lying entirely below the horizon row + margin."""
    px = np.zeros(len(am), dtype=int)
    nb = np.zeros(len(am), dtype=int)
    for i, frame in enumerate(am):
        lab, n = ndimage.label(frame)
        if n == 0:
            continue
        cut = min(frame.shape[0] - 1, int(hrows[i]) + HORIZON_MARGIN_ROWS)
        touching = set(np.unique(lab[:cut + 1])) - {0}
        keep = np.isin(lab, [k for k in range(1, n + 1) if k not in touching]) & (lab > 0)
        px[i] = int(keep.sum())
        nb[i] = n - len(touching)
    return px, nb


def blobs_per_frame(mask: np.ndarray) -> float:
    return float(np.mean([ndimage.label(f)[1] for f in mask]))


def persist_3of4(mask: np.ndarray) -> float:
    if len(mask) < 4:
        return float("nan")
    win = np.stack([mask[i:len(mask) - 3 + i] for i in range(4)])  # (4, N-3, H, W)
    stable = win.sum(axis=0) >= 3
    cur = mask[3:]
    denom = cur.sum()
    return float((stable & cur).sum() / denom) if denom else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--window", required=True)
    args = ap.parse_args()
    win = Path(args.window)
    src = load_sources(win)
    valid = src.pop("_valid")
    n = len(valid)

    hrows = horizon_rows(src["A_recorded"], valid)
    band = water_band(hrows, 96, 128)
    band_px = band.sum()
    print(f"{win.name}: {n} frames, horizon row (A) median {int(np.median(hrows))} "
          f"range {hrows.min()}..{hrows.max()}, water band = {band_px / n:.0f} px/frame")

    rows = []
    series: dict[str, np.ndarray] = {}
    for name, probs in src.items():
        masks = obstacle_masks(probs)
        rec = {"source": name, "frames": n}
        for rule, m in masks.items():
            rec[f"obst_frac[{rule}]"] = float(m.mean())
            rec[f"obst_frac_water[{rule}]"] = float((m & band).sum() / band_px) if band_px else float("nan")
        am = masks["argmax"]
        cls = probs.argmax(axis=-1)
        chg = cls[1:] != cls[:-1]
        rec["flicker"] = float(chg.mean())
        rec["flicker_water"] = float((chg & band[1:]).sum() / band[1:].sum()) if band[1:].sum() else float("nan")
        rec["blobs_water"] = blobs_per_frame(am & band)
        rec["persist_3of4_water"] = persist_3of4(am & band)
        ipx, inb = interior_components(am, hrows)
        rec["interior_px"] = float(ipx.mean())
        rec["interior_blobs"] = float(inb.mean())
        rec["interior_frames"] = float((inb > 0).mean())
        rows.append(rec)
        series[name] = ipx

    with open(win / "timeseries.csv", "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["idx", "horizon_row"] + [f"interior_px[{k}]" for k in series])
        for i in range(n):
            w.writerow([i, int(hrows[i])] + [int(series[k][i]) for k in series])

    keys = list(rows[0].keys())
    with open(win / "metrics.csv", "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    show = ["source", "obst_frac_water[argmax]", "interior_px", "interior_blobs", "interior_frames",
            "flicker_water", "persist_3of4_water"]
    print("  ".join(f"{k:>24}" for k in show))
    for r in rows:
        print("  ".join(f"{r[k]:>24}" if isinstance(r[k], str) else f"{r[k]:>24.4f}" for k in show))
    return 0


if __name__ == "__main__":
    sys.exit(main())
