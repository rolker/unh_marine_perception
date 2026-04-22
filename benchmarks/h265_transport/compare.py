#!/usr/bin/env python3
"""Compare a JPEG CompressedImage source bag to an H.265 FFMPEGPacket transcode
bag for the same camera. Produces per-frame and aggregate bandwidth, encode
latency, and SSIM/PSNR metrics.

Pairs frames by PTS (message order): the transcoder emits one encoded packet
per input frame, so the Nth FFMPEGPacket is compared against the Nth JPEG.

Used for the H.265 transport benchmark (rolker/unh_marine_perception#2).
"""

from __future__ import annotations

import argparse
import io
import os
import sys
from dataclasses import dataclass
from typing import Iterator

import av
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from PIL import Image as PILImage
from skimage.metrics import peak_signal_noise_ratio, structural_similarity


def find_mcap(path: str) -> str:
    if os.path.isfile(path) and path.endswith(".mcap"):
        return path
    if os.path.isdir(path):
        mcaps = [f for f in os.listdir(path) if f.endswith(".mcap")]
        if len(mcaps) == 1:
            return os.path.join(path, mcaps[0])
        raise ValueError(f"Expected exactly one .mcap in {path}; found {mcaps}")
    raise FileNotFoundError(path)


@dataclass
class BagStats:
    count: int
    total_bytes: int
    duration_s: float

    @property
    def bw_kbps(self) -> float:
        return self.total_bytes / self.duration_s / 1024 if self.duration_s > 0 else 0.0

    @property
    def bw_mbps(self) -> float:
        return self.bw_kbps * 8 / 1024


def iter_jpeg_frames(mcap_path: str, topic: str) -> Iterator[tuple[int, bytes]]:
    factory = DecoderFactory()
    with open(mcap_path, "rb") as fp:
        reader = make_reader(fp, decoder_factories=[factory])
        for _schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=[topic]
        ):
            yield message.log_time, bytes(ros_msg.data)


def iter_ffmpeg_packets(mcap_path: str, topic: str) -> Iterator[tuple[int, str, bytes, bool]]:
    factory = DecoderFactory()
    with open(mcap_path, "rb") as fp:
        reader = make_reader(fp, decoder_factories=[factory])
        for _schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=[topic]
        ):
            raw_encoding = getattr(ros_msg, "encoding", "") or ""
            codec = raw_encoding.split(";", 1)[0] if raw_encoding else ""
            is_key = bool(getattr(ros_msg, "flags", 0) & 0x1)
            yield message.log_time, codec, bytes(ros_msg.data), is_key


def measure_bag(mcap_path: str, topic: str) -> BagStats:
    """Compute per-topic stats using the source frame timestamps from the ROS
    Header, not rosbag2 log_time. This makes comparisons accurate regardless
    of the rosbag2 play_rate used during transcoding — a bag transcoded at
    10× still reports the real-time bandwidth that would appear on the wire
    at the camera's native 5 Hz."""
    count = 0
    total = 0
    t_min: float | None = None
    t_max: float | None = None
    factory = DecoderFactory()
    with open(mcap_path, "rb") as fp:
        reader = make_reader(fp, decoder_factories=[factory])
        for _schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=[topic]
        ):
            count += 1
            total += len(message.data)
            header = getattr(ros_msg, "header", None)
            if header is not None:
                stamp = header.stamp
                t = stamp.sec + stamp.nanosec / 1e9
            else:
                t = message.log_time / 1e9
            if t_min is None or t < t_min:
                t_min = t
            if t_max is None or t > t_max:
                t_max = t
    duration_s = (t_max - t_min) if t_min is not None and t_max is not None else 0.0
    return BagStats(count=count, total_bytes=total, duration_s=duration_s)


def decode_jpeg_frames(mcap_path: str, topic: str) -> Iterator[np.ndarray]:
    for _t, data in iter_jpeg_frames(mcap_path, topic):
        img = PILImage.open(io.BytesIO(data)).convert("RGB")
        yield np.asarray(img)


def decode_ffmpeg_frames(mcap_path: str, topic: str, codec_hint: str = "hevc") -> Iterator[np.ndarray]:
    codec = None
    context = None
    for _t, codec_str, data, _is_key in iter_ffmpeg_packets(mcap_path, topic):
        if context is None:
            codec_name = codec_str or codec_hint
            if codec_name.startswith("libx"):
                codec_name = {"libx265": "hevc", "libx264": "h264"}.get(codec_name, codec_name)
            codec = av.codec.Codec(codec_name, "r")
            context = codec.create()
            context.pix_fmt = "yuv420p"
        packet = av.Packet(data)
        for frame in context.decode(packet):
            rgb = frame.to_ndarray(format="rgb24")
            yield rgb
    if context is not None:
        for frame in context.decode(None):
            yield frame.to_ndarray(format="rgb24")


def compute_quality(ref_frames: Iterator[np.ndarray], test_frames: Iterator[np.ndarray]) -> list[tuple[float, float]]:
    scores: list[tuple[float, float]] = []
    for ref, test in zip(ref_frames, test_frames):
        if ref.shape != test.shape:
            continue
        ssim = structural_similarity(ref, test, channel_axis=2, data_range=255)
        psnr = peak_signal_noise_ratio(ref, test, data_range=255)
        scores.append((ssim, psnr))
    return scores


def percentile(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    return float(np.percentile(xs, p))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jpeg_bag", help="Path to source bag (JPEG CompressedImage)")
    parser.add_argument("ffmpeg_bag", help="Path to transcoded bag (FFMPEGPacket)")
    parser.add_argument("--jpeg-topic", required=True,
                        help="CompressedImage topic in jpeg_bag (fully qualified)")
    parser.add_argument("--ffmpeg-topic", required=True,
                        help="FFMPEGPacket topic in ffmpeg_bag (fully qualified)")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="Limit comparison to first N frames (0 = all)")
    parser.add_argument("--markdown", action="store_true",
                        help="Emit a Markdown summary row instead of aligned text")
    parser.add_argument("--label", default="",
                        help="Optional profile label to prefix Markdown output")
    args = parser.parse_args()

    jpeg_path = find_mcap(args.jpeg_bag)
    ffmpeg_path = find_mcap(args.ffmpeg_bag)

    jpeg_stats = measure_bag(jpeg_path, args.jpeg_topic)
    ffmpeg_stats = measure_bag(ffmpeg_path, args.ffmpeg_topic)

    ref_iter = decode_jpeg_frames(jpeg_path, args.jpeg_topic)
    test_iter = decode_ffmpeg_frames(ffmpeg_path, args.ffmpeg_topic)

    if args.max_frames > 0:
        import itertools
        ref_iter = itertools.islice(ref_iter, args.max_frames)
        test_iter = itertools.islice(test_iter, args.max_frames)

    scores = compute_quality(ref_iter, test_iter)

    if not scores:
        print("No frame pairs produced — check topic names or codec support.", file=sys.stderr)
        return 1

    ssims = [s for s, _ in scores]
    psnrs = [p for _, p in scores]

    ratio = jpeg_stats.bw_kbps / ffmpeg_stats.bw_kbps if ffmpeg_stats.bw_kbps > 0 else float("inf")

    if args.markdown:
        label = args.label or "profile"
        print(
            f"| {label} | {jpeg_stats.bw_mbps:.2f} | {ffmpeg_stats.bw_mbps:.2f} | "
            f"{ratio:.1f}x | {np.mean(ssims):.4f} | {percentile(ssims, 5):.4f} | "
            f"{np.mean(psnrs):.2f} | {percentile(psnrs, 5):.2f} | "
            f"{len(scores)} |"
        )
    else:
        print(f"JPEG source:   {jpeg_stats.count:5d} msgs, {jpeg_stats.bw_mbps:6.2f} Mbps ({jpeg_stats.bw_kbps:.1f} KB/s)")
        print(f"H.265 encoded: {ffmpeg_stats.count:5d} msgs, {ffmpeg_stats.bw_mbps:6.2f} Mbps ({ffmpeg_stats.bw_kbps:.1f} KB/s)")
        print(f"Bandwidth reduction: {ratio:.1f}x")
        print()
        print(f"Frame pairs compared: {len(scores)}")
        print(f"SSIM  mean={np.mean(ssims):.4f}  p50={percentile(ssims, 50):.4f}  p5={percentile(ssims, 5):.4f}  min={min(ssims):.4f}")
        print(f"PSNR  mean={np.mean(psnrs):6.2f}  p50={percentile(psnrs, 50):6.2f}  p5={percentile(psnrs, 5):6.2f}  min={min(psnrs):6.2f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
