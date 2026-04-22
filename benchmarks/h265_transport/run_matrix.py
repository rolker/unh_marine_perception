#!/usr/bin/env python3
"""Batch-run the H.265 transport benchmark matrix (rolker/unh_marine_perception#2).

Iterates 4 cameras × 7 (bitrate, GOP) profiles = 28 cells. For each cell:
  1. Invokes transcode_bag.launch.py to produce an H.265 bag.
  2. Invokes compare.py to compute real-time bandwidth + SSIM/PSNR vs JPEG.
  3. Appends one Markdown row to the running results file.

Play rate defaults to 10× for speed — the encoded bytes are the same as at
1× (verified by smoke tests), and compare.py uses source Header timestamps
for bandwidth so play_rate is neutral.

ROS 2 environment must be sourced before running this script.
"""

from __future__ import annotations

import argparse
import datetime
import os
import shutil
import subprocess
import sys
from pathlib import Path

CAMERAS = ("oak_forward", "oak_port", "oak_aft", "oak_starboard")

# (bitrate_kbps, gop_size_frames). GOP 30 dominates GOP 15 at the same target
# (identical bandwidth, marginally better SSIM) per the first matrix run, so
# the extended high-bitrate cells only sweep bitrate at GOP 30. The high end
# targets ~1 Mbps actual output per camera (matching today's throttled-JPEG
# budget of ~1 Mbps/camera at 0.5 Hz) so we can compare quality at equivalent
# bandwidth but 10× frame rate.
PROFILES: tuple[tuple[int, int], ...] = (
    (500, 5),
    (500, 15),
    (1000, 15),
    (1500, 15),
    (1500, 30),
    (2500, 30),
    (4000, 30),
    (6000, 30),
    (8000, 30),
    (12000, 30),
    (17000, 30),
    (20000, 30),
)

SCRIPT_DIR = Path(__file__).resolve().parent
LAUNCH_FILE = SCRIPT_DIR / "launch" / "transcode_bag.launch.py"
COMPARE_SCRIPT = SCRIPT_DIR / "compare.py"
VENV_PY = Path("/home/roland/project11/.venv/bin/python3")

HEADER = (
    "| Camera | Profile | Target kbps | GOP | JPEG Mbps | H.265 Mbps | Ratio | "
    "SSIM mean | SSIM p5 | PSNR mean | PSNR p5 | Frames |\n"
    "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"
)


def run_transcode(bag: Path, camera: str, bitrate_kbps: int, gop: int,
                  output_bag: Path, play_rate: float, log_file: Path) -> bool:
    input_topic = f"/bizzy/sensors/cameras/{camera}/image_raw"
    cmd = [
        "ros2", "launch", str(LAUNCH_FILE),
        f"bag:={bag}",
        f"input_topic:={input_topic}",
        f"output_bag:={output_bag}",
        f"bitrate:={bitrate_kbps * 1000}",
        f"gop_size:={gop}",
        f"play_rate:={play_rate}",
    ]
    with open(log_file, "w") as lf:
        result = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT)
    return result.returncode == 0 and output_bag.exists()


def run_compare(jpeg_bag: Path, ffmpeg_bag: Path, camera: str,
                bitrate_kbps: int, gop: int, max_frames: int) -> str:
    jpeg_topic = f"/bizzy/sensors/cameras/{camera}/image_raw/compressed"
    label = f"{camera} | b{bitrate_kbps}k g{gop} | {bitrate_kbps} | {gop}"
    cmd = [
        str(VENV_PY), str(COMPARE_SCRIPT),
        str(jpeg_bag), str(ffmpeg_bag),
        "--jpeg-topic", jpeg_topic,
        "--ffmpeg-topic", "/h265_bench/encoded/ffmpeg",
        "--markdown",
        "--label", label,
    ]
    if max_frames > 0:
        cmd += ["--max-frames", str(max_frames)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        return f"| {label} | - | - | - | - | - | - | - | - | **FAILED: {result.stderr.strip()[:80]}** |"
    return result.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_bag",
                        help="Path to source rosbag (JPEG CompressedImage bag)")
    parser.add_argument("--output-dir", default="/tmp/h265_matrix",
                        help="Directory for intermediate H.265 bags (default: /tmp/h265_matrix)")
    parser.add_argument("--results-file", default=None,
                        help="Markdown output path (default: results/matrix_<timestamp>.md)")
    parser.add_argument("--play-rate", type=float, default=10.0,
                        help="ros2 bag play rate (default: 10.0 for speed)")
    parser.add_argument("--max-frames", type=int, default=50,
                        help="Per-cell frames for SSIM/PSNR (default: 50; 0 = all)")
    parser.add_argument("--cameras", nargs="+", default=list(CAMERAS),
                        help=f"Cameras to benchmark (default: {' '.join(CAMERAS)})")
    parser.add_argument("--keep-bags", action="store_true",
                        help="Keep intermediate transcoded bags (default: delete after compare)")
    args = parser.parse_args()

    source_bag = Path(args.source_bag).expanduser().resolve()
    if not source_bag.exists():
        print(f"Source bag not found: {source_bag}", file=sys.stderr)
        return 1

    if shutil.which("ros2") is None:
        print("ros2 not in PATH — source setup.bash before running.", file=sys.stderr)
        return 1

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.results_file is None:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        results_file = SCRIPT_DIR / "results" / f"matrix_{ts}.md"
    else:
        results_file = Path(args.results_file).expanduser().resolve()
    results_file.parent.mkdir(parents=True, exist_ok=True)

    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=SCRIPT_DIR, text=True
        ).strip()
    except subprocess.CalledProcessError:
        commit = "unknown"

    total_cells = len(args.cameras) * len(PROFILES)
    header_lines = [
        "# H.265 benchmark matrix results",
        "",
        f"- Source bag: `{source_bag}`",
        f"- Run: {datetime.datetime.now().isoformat(timespec='seconds')}",
        f"- Commit: `{commit}`",
        f"- Play rate: {args.play_rate}× | Frames for SSIM/PSNR per cell: "
        f"{args.max_frames if args.max_frames > 0 else 'all'}",
        "",
        HEADER,
    ]
    with open(results_file, "w") as f:
        f.write("\n".join(header_lines) + "\n")

    print(f"Results: {results_file}")
    print(f"Running {total_cells} cells...")

    cell = 0
    for camera in args.cameras:
        for bitrate_kbps, gop in PROFILES:
            cell += 1
            label = f"{camera}_b{bitrate_kbps}k_g{gop}"
            bag_dir = output_dir / label
            log_file = output_dir / f"{label}.log"
            if bag_dir.exists():
                shutil.rmtree(bag_dir)

            print(f"[{cell}/{total_cells}] {label} ... ", end="", flush=True)
            ok = run_transcode(
                source_bag, camera, bitrate_kbps, gop,
                bag_dir, args.play_rate, log_file
            )
            if not ok:
                row = (f"| {camera} | b{bitrate_kbps}k g{gop} | {bitrate_kbps} | {gop} | "
                       f"- | - | - | - | - | - | - | **TRANSCODE FAILED** |")
                print("FAILED (transcode)")
            else:
                row = run_compare(
                    source_bag, bag_dir, camera, bitrate_kbps, gop, args.max_frames
                )
                if "FAILED" in row:
                    print("FAILED (compare)")
                else:
                    print("ok")

            with open(results_file, "a") as f:
                f.write(row + "\n")

            if not args.keep_bags and bag_dir.exists():
                shutil.rmtree(bag_dir)

    print(f"\nDone. Results: {results_file}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
