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
├── launch/
│   └── transcode_bag.launch.py   # play bag -> decode JPEG -> encode H.265 -> record
├── baselines/
│   └── bizzy_images_2026-04-21.md   # JPEG baseline from the reference bags
└── (compare.py)             # TODO: SSIM/PSNR comparison
    (results/)               # TODO: matrix results
```

## Dependencies

- Python 3 with the `mcap` package — use the workspace venv at
  `/home/roland/project11/.venv/bin/python3`, which has it installed.
- `ffmpeg_image_transport` — declared as an `exec_depend` in
  `depthai_marine/package.xml`. Install via `rosdep install` across the
  sensors layer, or directly:
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

### Transcode a bag

Plays a bag, decodes one `CompressedImage` topic, re-encodes with H.265 via
`ffmpeg_image_transport`, and records the resulting `FFMPEGPacket` stream to
a new bag:

```bash
ros2 launch benchmarks/h265_transport/launch/transcode_bag.launch.py \
  bag:=~/data/logs/bizzy_images/bag_2026-04-21T13.58.31 \
  input_topic:=/bizzy/sensors/cameras/oak_forward/image_raw \
  output_bag:=/tmp/h265_forward_b1500k_g15 \
  bitrate:=1500000 \
  gop_size:=15
```

- `input_topic` is the **base** name — the launch file appends `/compressed`
  to match what's in the bag.
- The launch shuts down automatically when `ros2 bag play` exits.
- libx265 is constrained to HW-mimic flags; see the node parameters in the
  launch file for the full `x265-params` string.

### Baseline

See [`baselines/bizzy_images_2026-04-21.md`](baselines/bizzy_images_2026-04-21.md)
for the current JPEG baseline that H.265 is competing against. TL;DR: the four
OAK cameras on bizzyboat use **33–42 Mbps** of aggregate JPEG bandwidth
depending on scene; target for H.265 is roughly **2–4 Mbps** at equivalent
quality.

## OAK hardware encoder constraints — and what we can enforce on libx265

The Myriad X `VideoEncoder` is:

- H.265 **Main profile** only (no Main10, no Main-Intra).
- No B-frames, single reference frame.
- No look-ahead / no adaptive quantization / no scene-change detection.
- CBR or VBR (no CRF).
- NV12 input, width multiple of 32 (1280x720 complies).

What the `ffmpeg_image_transport` parameter parser actually lets us pass to
libx265:

| HW constraint | Enforced? | How |
|---|---|---|
| Main profile | Yes | `profile:main` |
| No B-frames | Yes | `max_b_frames:0` (libav AVOption on AVCodecContext) |
| Single reference | Yes | `refs:1` (libav AVOption on AVCodecContext) |
| Fixed keyframe interval | Yes | top-level `gop_size` param |
| No look-ahead | **No** | requires `x265-params:rc-lookahead=0` |
| No scene-change keyframes | **No** | requires `x265-params:scenecut=0` |
| No adaptive quantization | **No** | requires `x265-params:aq-mode=0` |
| No cutree | **No** | requires `x265-params:cutree=0` |
| Strict CBR / VBV tuning | **No** | requires `x265-params:rc=cbr:strict-cbr=1:vbv-*` |

Why "No" for the bottom rows: `ffmpeg_image_transport`'s `encoder_av_options`
parser is `key:value,key:value,...` and does not accept nested `:` in values.
The `x265-params:...` string (whose value itself uses `:` as a separator) is
rejected with "skipping bad AV option" at encoder init. This means libx265
retains more rate-distortion optimization than the Myriad X hardware will
have, so **software results will be optimistic** vs. the eventual on-device
encoder — probably 10–20% smaller at matched visual quality (the 5–15% we
already expected from RDO, compounded by the remaining lookahead/AQ/cutree
differences). Calibrate by running one clip through an actual OAK when
hardware is available.

Also note: `pixel_format: yuv420p` (not `nv12`, which the Myriad X wants);
libx265 software does not accept `nv12`. This is a colourspace-conversion
step that the OAK HW pipeline skips.

Sources:
- [`VideoEncoderProperties.hpp`](https://github.com/luxonis/depthai-core/blob/main/include/depthai/properties/VideoEncoderProperties.hpp)
- [Luxonis VideoEncoder docs](https://docs.luxonis.com/software/depthai-components/nodes/video_encoder/)

Sources:
- [`VideoEncoderProperties.hpp`](https://github.com/luxonis/depthai-core/blob/main/include/depthai/properties/VideoEncoderProperties.hpp)
- [Luxonis VideoEncoder docs](https://docs.luxonis.com/software/depthai-components/nodes/video_encoder/)

## Out of scope

- UDP bridge transport / link emulation.
- Modifying `depthai_marine` to add on-device `VideoEncoder`.
- Runtime / launch-config changes on bizzyboat.
- Hardware-accelerated encoders (`hevc_nvenc`, `hevc_vaapi`).
- H.264 or MJPEG comparison (possible follow-ups).
