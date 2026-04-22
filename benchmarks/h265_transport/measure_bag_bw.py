#!/usr/bin/env python3
"""Print per-topic bandwidth (bytes/sec), average message size, count, and rate
for a rosbag2 MCAP bag. Used to establish baseline numbers for the H.265
transport benchmark (rolker/unh_marine_perception#2)."""

from __future__ import annotations

import argparse
import os
import re
import sys
from collections import defaultdict

from mcap.reader import make_reader


def find_mcap(path: str) -> str:
    if os.path.isfile(path) and path.endswith(".mcap"):
        return path
    if os.path.isdir(path):
        mcaps = [f for f in os.listdir(path) if f.endswith(".mcap")]
        if len(mcaps) == 1:
            return os.path.join(path, mcaps[0])
        if len(mcaps) == 0:
            raise FileNotFoundError(f"No .mcap file found in {path}")
        raise ValueError(
            f"Multiple .mcap files in {path} — pass one explicitly: {mcaps}"
        )
    raise FileNotFoundError(path)


def measure(mcap_path: str, topic_re: re.Pattern | None):
    bytes_by: dict[str, int] = defaultdict(int)
    count_by: dict[str, int] = defaultdict(int)
    type_by: dict[str, str] = {}
    t_min: int | None = None
    t_max: int | None = None

    with open(mcap_path, "rb") as fp:
        reader = make_reader(fp)
        for schema, channel, message in reader.iter_messages():
            if topic_re and not topic_re.search(channel.topic):
                continue
            bytes_by[channel.topic] += len(message.data)
            count_by[channel.topic] += 1
            if schema is not None:
                type_by[channel.topic] = schema.name
            if t_min is None or message.log_time < t_min:
                t_min = message.log_time
            if t_max is None or message.log_time > t_max:
                t_max = message.log_time

    duration_s = (
        (t_max - t_min) / 1e9
        if t_min is not None and t_max is not None
        else 0.0
    )
    return bytes_by, count_by, type_by, duration_s


def format_row(topic: str, count: int, total_bytes: int, duration_s: float, msg_type: str) -> str:
    rate = count / duration_s if duration_s > 0 else 0.0
    avg_kb = (total_bytes / count / 1024) if count else 0.0
    kbps = (total_bytes / duration_s / 1024) if duration_s > 0 else 0.0
    return (
        f"  {count:6d}  {rate:6.2f} Hz  avg={avg_kb:8.2f} KB  "
        f"bw={kbps:9.1f} KB/s  {msg_type:36s}  {topic}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Path to rosbag2 directory or .mcap file")
    parser.add_argument(
        "--topic-regex",
        "-t",
        help="Only include topics matching this regex (default: all)",
    )
    parser.add_argument(
        "--markdown",
        "-m",
        action="store_true",
        help="Emit a Markdown table instead of aligned text",
    )
    parser.add_argument(
        "--min-bw-kbps",
        type=float,
        default=0.0,
        help="Hide topics below this bandwidth (KB/s); default 0 = show all",
    )
    args = parser.parse_args()

    mcap_path = find_mcap(args.bag)
    topic_re = re.compile(args.topic_regex) if args.topic_regex else None
    bytes_by, count_by, type_by, duration_s = measure(mcap_path, topic_re)

    if not bytes_by:
        print("No matching topics found.", file=sys.stderr)
        return 1

    rows = sorted(
        bytes_by.items(), key=lambda kv: kv[1] / (duration_s or 1.0), reverse=True
    )
    rows = [
        (t, b) for t, b in rows
        if (b / (duration_s or 1.0) / 1024) >= args.min_bw_kbps
    ]

    print(f"# Bag: {mcap_path}")
    print(f"# Duration: {duration_s:.2f} s, topics: {len(rows)}")

    if args.markdown:
        print()
        print("| Topic | Type | Count | Rate (Hz) | Avg (KB) | BW (KB/s) | BW (Mbps) |")
        print("|---|---|---:|---:|---:|---:|---:|")
        for topic, total in rows:
            c = count_by[topic]
            rate = c / duration_s if duration_s > 0 else 0.0
            avg_kb = (total / c / 1024) if c else 0.0
            kbps = total / duration_s / 1024
            mbps = kbps * 8 / 1024
            msg_type = type_by.get(topic, "?")
            print(
                f"| `{topic}` | `{msg_type}` | {c} | {rate:.2f} | "
                f"{avg_kb:.2f} | {kbps:.1f} | {mbps:.2f} |"
            )
    else:
        print()
        for topic, total in rows:
            print(format_row(topic, count_by[topic], total, duration_s, type_by.get(topic, "?")))

    total_all = sum(b for _, b in rows)
    total_kbps = total_all / duration_s / 1024 if duration_s > 0 else 0.0
    print()
    print(f"# Total (shown topics): {total_kbps:.1f} KB/s ({total_kbps * 8 / 1024:.2f} Mbps)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
