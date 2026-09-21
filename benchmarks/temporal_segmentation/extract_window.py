#!/usr/bin/env python3
"""Extract a time window of one OAK camera from a `bizzy_images` camera bag.

Decodes the H.265 `image_raw/ffmpeg` stream, stretches every frame to the
512x384 NN input geometry the on-camera pipeline uses (anisotropic, full FOV —
see sea_surface_segmentation.cpp, ImageManip setKeepAspectRatio(false)), and
pairs each frame with the recorded on-camera eWaSR softmax mask
(`segmentation`, 128x96 rgb8: R=obstacle, G=water, B=sky) by header stamp.

Output (one directory per window+camera):
  frames.npz   uint8 (N,384,512,3) RGB   decoded + stretched frames
  masks.npz    uint8 (N,96,128,3)  RGB   recorded on-camera eWaSR softmax*255
  index.csv    idx, frame_stamp_ns, mask_stamp_ns, pair_dt_ms

Frames with no mask within --pair-tol-ms are kept with mask_stamp_ns = -1 and
an all-zero mask row, so the sequence stays gapless for the temporal model.

Part of rolker/unh_marine_perception#49 (evidence for #42).
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

import av
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from PIL import Image

NN_W, NN_H = 512, 384
SEG_W, SEG_H = 128, 96
# Longest configured H.265 GOP on the boat is 37 frames @ 5 fps = 7.4 s; start
# decoding this far ahead of the window so the first wanted frame follows a
# keyframe.
GOP_LEAD_S = 10.0


def find_mcap(path: str) -> str:
    p = Path(path)
    if p.is_file() and p.suffix == ".mcap":
        return str(p)
    if p.is_dir():
        mcaps = sorted(p.glob("*.mcap"))
        if len(mcaps) == 1:
            return str(mcaps[0])
        raise ValueError(f"expected exactly one .mcap in {path}; found {[m.name for m in mcaps]}")
    raise FileNotFoundError(path)


def parse_time(s: str) -> int:
    """Local-time ISO string (bag names and deployment logs are local) or epoch
    seconds -> epoch nanoseconds."""
    try:
        return int(float(s) * 1e9)
    except ValueError:
        pass
    dt = datetime.fromisoformat(s)
    if dt.tzinfo is None:
        dt = dt.astimezone()  # interpret as local time
    return int(dt.timestamp() * 1e9)


def stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="camera bag directory or .mcap")
    ap.add_argument("--camera", required=True, choices=["oak_forward", "oak_port", "oak_starboard", "oak_aft"])
    ap.add_argument("--start", required=True, help="window start, local ISO (2026-06-03T15:50:00) or epoch s")
    ap.add_argument("--end", required=True, help="window end, same forms")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--namespace", default="/bizzy")
    ap.add_argument("--pair-tol-ms", type=float, default=120.0,
                    help="max |frame stamp - mask stamp| to pair (5 fps => 200 ms period)")
    ap.add_argument("--save-png", action="store_true", help="also write frames/NNNNN.png (512x384)")
    args = ap.parse_args()

    mcap_path = find_mcap(args.bag)
    t0, t1 = parse_time(args.start), parse_time(args.end)
    if t1 <= t0:
        print("error: --end must be after --start", file=sys.stderr)
        return 2
    vt = f"{args.namespace}/sensors/cameras/{args.camera}/image_raw/ffmpeg"
    st = f"{args.namespace}/sensors/cameras/{args.camera}/segmentation"

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # Pass 1: masks in window (small).
    masks: list[tuple[int, np.ndarray]] = []
    with open(mcap_path, "rb") as fp:
        reader = make_reader(fp, decoder_factories=[DecoderFactory()])
        for _s, _ch, _m, msg in reader.iter_decoded_messages(topics=[st], start_time=t0, end_time=t1):
            if msg.encoding != "rgb8" or msg.width != SEG_W or msg.height != SEG_H:
                print(f"error: unexpected mask format {msg.encoding} {msg.width}x{msg.height}", file=sys.stderr)
                return 1
            a = np.frombuffer(bytes(msg.data), np.uint8).reshape(SEG_H, SEG_W, 3).copy()
            masks.append((stamp_ns(msg.header), a))
    masks.sort(key=lambda x: x[0])
    mask_stamps = np.array([m[0] for m in masks], dtype=np.int64)

    # Pass 2: decode video from GOP_LEAD_S before the window.
    frames: list[np.ndarray] = []
    frame_stamps: list[int] = []
    pts_to_stamp: dict[int, int] = {}
    codec = None
    n_pkts = n_decoded = 0
    lead_start = t0 - int(GOP_LEAD_S * 1e9)

    def take(frame: av.VideoFrame) -> None:
        nonlocal n_decoded
        n_decoded += 1
        s = pts_to_stamp.get(frame.pts)
        if s is None or s < t0 or s >= t1:
            return
        img = frame.to_image()  # PIL RGB
        img = img.resize((NN_W, NN_H), Image.BILINEAR)  # anisotropic stretch, as on-camera
        frames.append(np.asarray(img, dtype=np.uint8))
        frame_stamps.append(s)

    with open(mcap_path, "rb") as fp:
        reader = make_reader(fp, decoder_factories=[DecoderFactory()])
        for _s, _ch, _m, msg in reader.iter_decoded_messages(topics=[vt], start_time=lead_start, end_time=t1):
            n_pkts += 1
            if codec is None:
                codec = av.CodecContext.create(msg.encoding, "r")  # "hevc"
            pkt = av.Packet(bytes(msg.data))
            pkt.pts = int(msg.pts)
            pts_to_stamp[int(msg.pts)] = stamp_ns(msg.header)
            for f in codec.decode(pkt):
                take(f)
        if codec is not None:
            for f in codec.decode(None):  # flush
                take(f)

    if not frames:
        print(f"error: no decodable frames in window ({n_pkts} packets read)", file=sys.stderr)
        return 1

    order = np.argsort(np.array(frame_stamps, dtype=np.int64))
    frames_arr = np.stack([frames[i] for i in order])
    fstamps = np.array([frame_stamps[i] for i in order], dtype=np.int64)

    # Pair each frame with the nearest mask.
    paired = np.zeros((len(fstamps), SEG_H, SEG_W, 3), dtype=np.uint8)
    mstamp_col = np.full(len(fstamps), -1, dtype=np.int64)
    dt_col = np.full(len(fstamps), np.nan)
    tol_ns = int(args.pair_tol_ms * 1e6)
    unpaired = 0
    for i, fs in enumerate(fstamps):
        if len(mask_stamps) == 0:
            unpaired += 1
            continue
        j = int(np.searchsorted(mask_stamps, fs))
        cands = [k for k in (j - 1, j) if 0 <= k < len(mask_stamps)]
        k = min(cands, key=lambda k: abs(int(mask_stamps[k]) - int(fs)))
        d = int(mask_stamps[k]) - int(fs)
        if abs(d) <= tol_ns:
            paired[i] = masks[k][1]
            mstamp_col[i] = mask_stamps[k]
            dt_col[i] = d / 1e6
        else:
            unpaired += 1

    np.savez_compressed(out / "frames.npz", frames=frames_arr, stamp_ns=fstamps)
    np.savez_compressed(out / "masks.npz", masks=paired, stamp_ns=mstamp_col)
    with open(out / "index.csv", "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["idx", "frame_stamp_ns", "mask_stamp_ns", "pair_dt_ms"])
        for i in range(len(fstamps)):
            w.writerow([i, int(fstamps[i]), int(mstamp_col[i]), "" if np.isnan(dt_col[i]) else f"{dt_col[i]:.1f}"])
    if args.save_png:
        d = out / "frames"
        d.mkdir(exist_ok=True)
        for i in range(len(fstamps)):
            Image.fromarray(frames_arr[i]).save(d / f"{i:05d}.png")

    span_s = (int(fstamps[-1]) - int(fstamps[0])) / 1e9 if len(fstamps) > 1 else 0.0
    local0 = datetime.fromtimestamp(int(fstamps[0]) / 1e9).isoformat(timespec="seconds")
    print(f"{args.camera}: {len(fstamps)} frames over {span_s:.1f}s from {local0} "
          f"({n_pkts} packets, {n_decoded} decoded incl. lead-in); "
          f"{len(masks)} masks in window, {unpaired} frames unpaired -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
