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
├── requirements.txt         # Python analysis deps (PyAV, scikit-image)
├── measure_bag_bw.py        # per-topic bandwidth from an MCAP bag
├── compare.py               # JPEG vs H.265 bandwidth + SSIM/PSNR
├── launch/
│   └── transcode_bag.launch.py   # play bag -> decode JPEG -> encode H.265 -> record
├── baselines/
│   └── bizzy_images_2026-04-21.md   # JPEG baseline from the reference bags
└── (results/)               # TODO: matrix results
```

## Dependencies

- **ROS runtime**: `ffmpeg_image_transport` — declared as an `exec_depend` in
  `depthai_marine/package.xml`. Install via `rosdep install` across the
  sensors layer, or directly:
  `sudo apt install ros-jazzy-ffmpeg-image-transport`.
- **Python tools** (measure / compare): use the workspace venv at
  `/home/roland/project11/.venv/bin/python3`. `mcap` is already there; install
  the remaining analysis deps:
  ```bash
  /home/roland/project11/.venv/bin/pip install -r benchmarks/h265_transport/requirements.txt
  ```
  This adds `av` (PyAV, for H.265 decode from FFMPEGPacket bags) and
  `scikit-image` (for SSIM/PSNR).

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

### Compare a transcoded bag to its JPEG source

```bash
.venv/bin/python3 benchmarks/h265_transport/compare.py \
  ~/data/logs/bizzy_images/bag_2026-04-21T13.58.31 \
  /tmp/h265_smoke \
  --jpeg-topic /bizzy/sensors/cameras/oak_forward/image_raw/compressed \
  --ffmpeg-topic /h265_bench/encoded/ffmpeg
```

Reports per-camera bandwidth (real-time, derived from the source frame
Header timestamps so it's accurate regardless of `play_rate` used during
transcoding), bandwidth reduction ratio, and SSIM / PSNR vs the JPEG source.

Flags:

- `--max-frames N` — limit comparison to first N frames (fast sanity check).
- `--markdown --label "<profile>"` — emit a single Markdown table row per
  profile, for rolling up the 28-cell matrix.

### Caveat on `bit_rate` as a control knob

At `preset=ultrafast` with no `rc-lookahead` (which we're forced into because
the HW mimic can't run lookahead anyway), libx265's ABR rate control is very
loose on short clips. Setting `bitrate:=1500000` typically produces a bag at
**~90 kbps**, not 1.5 Mbps — but at 500 kbps target, the bag drops to ~30 kbps
(proportional). The `bit_rate` param is a working control, just not a literal
target in this configuration. For the matrix, report *measured* bandwidth
from the bag, not the target. The Myriad X hardware encoder's rate control
may hit its target more precisely — this is another calibration item for
when OAK hardware is available.

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

## Results (first matrix run, 2026-04-21)

Full 28-cell run from source bag `bag_2026-04-21T13.58.31` lives in
[`results/matrix_20260421_223215.md`](results/matrix_20260421_223215.md).
Headlines across the 4 cameras (ranges span the 4 cameras at the same target):

| Target kbps | GOP | H.265 Mbps | Reduction | SSIM mean | PSNR mean |
|---:|---:|---:|---:|---:|---:|
|  500 |  5 | 0.03–0.05 | 187–222× | 0.62–0.67 | 25.0–26.2 |
|  500 | 15 | 0.02–0.03 | 267–336× | 0.61–0.66 | 24.7–26.0 |
| 1000 | 15 | 0.04–0.06 | 143–167× | 0.64–0.70 | 25.7–27.4 |
| 1500 | 15 | 0.06–0.08 | 100–117× | 0.68–0.73 | 26.5–28.1 |
| 1500 | 30 | 0.06–0.08 | 106–123× | 0.69–0.73 | 26.6–28.1 |
| 2500 | 30 | 0.09–0.13 |  66–76×  | 0.73–0.78 | 27.6–29.2 |
| 4000 | 30 | 0.14–0.20 |  43–48×  | 0.79–0.82 | 28.8–30.3 |

Against the four-camera JPEG aggregate of ~33 Mbps, the 4 Mbps / GOP-30 target
would consume ~0.75 Mbps — a very compelling operating point on wire bytes.

### Observations

- **GOP 30 dominates GOP 15 at the same bitrate target** — identical bandwidth
  (~0.08 Mbps at 1500k target), slightly better SSIM. Keyframes every 6 s at
  5 Hz is the right default.
- **GOP 5 at 500 kbps produces more bytes than GOP 15 at 500 kbps**, because
  keyframes are expensive and shorter GOPs means more of them. For
  bandwidth-constrained links, long GOPs help — at the cost of recovery time
  after packet loss.
- **SSIM 0.81 at the 4 Mbps target** is "visibly degraded but usable" (0.95+
  is visually identical). Below 0.70 the output is noticeably smeared,
  especially on horizon and surface-texture detail. For situational awareness,
  SSIM ≥ 0.75 looks like the floor.
- **The 17× gap between requested and delivered bitrate is consistent across
  all 28 cells**, confirming this is libx265's rate-control behavior under
  `preset=ultrafast` + `rc-lookahead=0`, not a pipeline bug. The Myriad X's
  rate control may be more precise — another calibration item.
- **oak_aft has the highest baseline** (9.7 Mbps JPEG) and consumes the most
  H.265 bytes; **oak_starboard the lowest** (5.9 Mbps) partly because its
  capture rate is 3.5 Hz not 5 Hz.

### Recommended operating point for the on-device encoder test

**Target: ~4 Mbps, GOP 30** (keyframes every 6 s at 5 Hz). This produced
SSIM 0.79–0.82 across cameras, consuming ~0.75 Mbps aggregate on wire —
enough headroom on any field link, and visually good enough for operator
situational awareness.

When an OAK camera is available, reproduce the cell at 4 Mbps / GOP 30 on
hardware and check whether actual bandwidth matches target more closely than
the software equivalent. If so, a HW target of 1–1.5 Mbps may hit equivalent
SSIM at even lower bandwidth.

## Out of scope

- UDP bridge transport / link emulation.
- Modifying `depthai_marine` to add on-device `VideoEncoder`.
- Runtime / launch-config changes on bizzyboat.
- Hardware-accelerated encoders (`hevc_nvenc`, `hevc_vaapi`).
- H.264 or MJPEG comparison (possible follow-ups).
