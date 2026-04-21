# H.265 transport benchmark

Tracks [rolker/unh_marine_perception#2](https://github.com/rolker/unh_marine_perception/issues/2).

Measures whether `ffmpeg_image_transport` H.265 — configured to mimic the
constraints of the OAK `VideoEncoder` hardware — can replace the current
per-frame JPEG transport of OAK camera streams with meaningful bandwidth
savings at comparable visual quality.

## Layout

```
benchmarks/h265_transport/
├── README.md                # this file
├── measure_bag_bw.py        # per-topic bandwidth from an MCAP bag
├── baselines/
│   └── bizzy_images_2026-04-21.md   # JPEG baseline from the reference bags
└── (transcode_bag.launch.py)        # TODO
    (compare.py)                     # TODO
    (results/)                       # TODO
```

## Dependencies

- Python 3 with the `mcap` package — use the workspace venv at
  `/home/roland/project11/.venv/bin/python3`, which has it installed.
- For transcoding (not yet added):
  `sudo apt install ros-jazzy-ffmpeg-image-transport`.

## Usage

### Measure any bag

```bash
.venv/bin/python3 benchmarks/h265_transport/measure_bag_bw.py \
  ~/data/logs/bizzy_images/bag_2026-04-21T13.58.31 \
  --topic-regex '(image_raw/compressed|segmentation/compressed)$' \
  --markdown
```

Flags:

- `--topic-regex PATTERN` — only include topics matching the regex (default: all).
- `--markdown` — Markdown table instead of aligned text.
- `--min-bw-kbps N` — hide topics below N KB/s.

### Baseline

See [`baselines/bizzy_images_2026-04-21.md`](baselines/bizzy_images_2026-04-21.md)
for the current JPEG baseline that H.265 is competing against. TL;DR: the four
OAK cameras on bizzyboat use **33–42 Mbps** of aggregate JPEG bandwidth
depending on scene; target for H.265 is roughly **2–4 Mbps** at equivalent
quality.

## OAK hardware encoder constraints

The Myriad X `VideoEncoder` is constrained to:

- H.265 **Main profile** only (no Main10, no Main-Intra).
- No B-frames, single reference frame.
- No look-ahead / no adaptive quantization / no scene-change detection.
- CBR or VBR (no CRF).
- NV12 input, width multiple of 32 (1280x720 complies).

Software `libx265` must be restricted to match, or the Phase 1 numbers won't
predict Phase 2 behavior. See the issue body for the full flag set.

Sources:
- [`VideoEncoderProperties.hpp`](https://github.com/luxonis/depthai-core/blob/main/include/depthai/properties/VideoEncoderProperties.hpp)
- [Luxonis VideoEncoder docs](https://docs.luxonis.com/software/depthai-components/nodes/video_encoder/)

## Out of scope

- UDP bridge transport / link emulation.
- Modifying `depthai_marine` to add on-device `VideoEncoder`.
- Runtime / launch-config changes on bizzyboat.
- Hardware-accelerated encoders (`hevc_nvenc`, `hevc_vaapi`).
- H.264 or MJPEG comparison (possible follow-ups).
