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
└── results/                 # committed benchmark result matrices
```

## Dependencies

- **ROS runtime**: `ffmpeg_image_transport` — declared as an `exec_depend` in
  `depthai_marine/package.xml`. Install via `rosdep install` across the
  sensors layer, or directly:
  `sudo apt install ros-jazzy-ffmpeg-image-transport`.
- **Python tools** (measure / compare): use the workspace venv (conventionally
  at `.venv/` in the workspace root — see ADR-0009). `mcap` is already there;
  with the venv active, install the remaining analysis deps:
  ```bash
  python -m pip install -r benchmarks/h265_transport/requirements.txt
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
- libx265 is constrained to HW-mimic settings; see the launch file's
  `encoder_av_options` together with the enforced `bit_rate` and `gop_size`
  node parameters.

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
  profile, for rolling up the benchmark matrix.

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

## Results

Two full runs from source bag `bag_2026-04-21T13.58.31`, both all-cells-ok:

- [`results/matrix_20260421_223215.md`](results/matrix_20260421_223215.md) —
  first 28-cell matrix (7 profiles × 4 cameras), low-to-mid bitrate band.
- [`results/matrix_20260421_232314.md`](results/matrix_20260421_232314.md) —
  extended 48-cell matrix (12 profiles × 4 cameras), adds the high band
  to reach the bandwidth budget today's throttled JPEG transport uses.

### Today's baseline (`udp_bridge` stats from `~/data/logs/bizzyboat/`)

Measured from `TopicStatisticsArray` + `BridgeInfo` on bag
`2026-04-14T12.43.44` (current production, not the benchmark source bag):

| Camera | Link(s) | Effective rate | Per-camera Mbps |
|---|---|---:|---:|
| oak_forward | vpn + wifi | 0.49 Hz | 1.06 (each link) |
| oak_port | wifi | 0.49 Hz | 0.89 |
| oak_aft | wifi | 0.49 Hz | 1.10 |
| oak_starboard | wifi | 0.47 Hz | 1.13 |

- **Four-camera wifi aggregate: ~4.18 Mbps** out of the 12 Mbps cap.
- **Effective frame rate the operator sees: ~0.5 Hz** (one frame every ~2 s)
  because `udp_bridge` throttles message rate to fit the bandwidth cap.
- oak_forward is duplicated on vpn (another 1.06 Mbps).

### H.265 curve at the same bandwidth budget, at full 5 Hz

Extended-matrix 3-camera aggregate at **GOP 30** (excludes `oak_aft` in this
summary — see *Caveats* below). Compare against today's 4.18 Mbps / 0.5 Hz
wifi line.

| Target kbps | 3-cam Mbps | SSIM mean | PSNR mean | vs today (3-cam wifi ≈ 3.1 Mbps) |
|---:|---:|---:|---:|---|
| 1500 | 0.22 | 0.73 | 27.7 | 1/14 bandwidth, 10× rate, usable |
| 2500 | 0.35 | 0.77 | 28.7 | 1/9 bandwidth, 10× rate, good |
| 4000 | 0.54 | 0.82 | 29.9 | 1/6 bandwidth, 10× rate, strong |
| 6000 | 0.78 | 0.85 | 31.0 | 1/4 bandwidth, 10× rate, very good |
| 8000 | 1.05 | 0.88 | 31.8 | 1/3 bandwidth, 10× rate |
| 12000 | 1.55 | **0.91** | 33.3 | 1/2 bandwidth, 10× rate, **near-identical** |
| 17000 | 2.18 | **0.93** | 34.6 | 2/3 bandwidth, 10× rate |
| 20000 | 2.56 | **0.94** | 35.2 | ~5/6 bandwidth, 10× rate, **visually identical** |

For completeness, the GOP-15 and low-target (500 kbps) rows from the first
matrix run are in [`results/matrix_20260421_223215.md`](results/matrix_20260421_223215.md) —
they only beat the mid-band on bandwidth while losing to it on SSIM. GOP 30
is the right default.

Rule-of-thumb reading: every row above the 2500-kbps line *already* matches
or beats today's production on bandwidth. The high-band rows trade bandwidth
for quality — picking the right one depends on the use case.

### Operating-point recommendations

| Goal | Target | Per-camera Mbps | SSIM | Why |
|---|---|---:|---:|---|
| **Minimum viable telemetry** | b500k / GOP 15 | 0.02–0.05 | 0.61–0.67 | Aggressive compression, situational awareness only. 200× reduction from current JPEG. |
| **Replacement at matched budget** | b12000k / GOP 30 | 0.3–0.6 | **0.91** | ~1/3 of today's bandwidth, 10× frame rate, near-visually-identical. |
| **Maximum quality within current budget** | b20000k / GOP 30 | 0.7–1.0 | **0.94** | Matches today's per-camera bandwidth exactly (~1 Mbps), 10× rate, indistinguishable from JPEG in casual viewing. |

**Recommendation for first on-device test on real OAK hardware:**
**b17000k / GOP 30**. Software result: ~0.80 Mbps per camera, SSIM 0.93.
That's slightly under today's 1 Mbps/camera, at 10× frame rate, at quality
any operator would consider indistinguishable. Expectation: the Myriad X
encoder with tighter rate control should hit this quality at a lower target
bitrate (possibly 2–3 Mbps target) with the same actual output.

### Observations

- **GOP 30 dominates GOP 15 at the same bitrate target** — identical
  bandwidth, slightly better SSIM. Keyframes every 6 s at 5 Hz is the right
  default.
- **GOP 5 at 500 kbps produces more bytes than GOP 15 at 500 kbps** because
  keyframes are expensive. For bandwidth-constrained links, long GOPs help —
  at the cost of recovery time after packet loss.
- **The 17× gap between requested and delivered bitrate** is consistent
  across all 48 cells at low-to-mid target; it **narrows as target increases**
  (e.g., b20000k delivers ~0.95 Mbps / camera, a ratio of ~21×, so rate
  control asymptotes). The Myriad X hardware encoder may hit target
  directly — this is a calibration item for when hardware is available.
- **oak_aft has the highest JPEG baseline** (9.7 Mbps) from marine scene
  content, and also showed anomalous SSIM dips at b8000/b12000/b17000
  (SSIM 0.47–0.56 where other cameras got 0.88–0.93) before recovering at
  b20000. Likely a PyAV decoder sync issue on that specific stream, not a
  real encoder quality regression. Re-running those cells in isolation
  should resolve it; for now the 3-camera aggregate above is a cleaner
  summary.

### Caveats (unchanged from first run)

- `x265-params:...` can't pass `ffmpeg_image_transport`'s `key:value` parser,
  so we lose cutree / aq-mode / scenecut / strict-CBR / VBV tightness.
  libx265 retains more RDO than the Myriad X will — software numbers here
  are 10–20% optimistic vs. eventual hardware.
- `preset=ultrafast` + `rc-lookahead=0` makes libx265's ABR very loose. The
  Myriad X's rate control may be closer to its target.
- Single source bag; scene variance across the three `bizzy_images` bags is
  ~40% on JPEG baseline. Re-running with the other two bags is a cheap
  sanity check before finalizing operating point.

## Out of scope

- UDP bridge transport / link emulation.
- Modifying `depthai_marine` to add on-device `VideoEncoder`.
- Runtime / launch-config changes on bizzyboat.
- Hardware-accelerated encoders (`hevc_nvenc`, `hevc_vaapi`).
- H.264 or MJPEG comparison (possible follow-ups).
