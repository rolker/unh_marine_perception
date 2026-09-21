# Temporal segmentation benchmark: WaSR-T vs eWaSR on recorded footage

Offline comparison of [WaSR-T](https://github.com/lojzezust/WaSR-T) (Žust &
Kristan, IROS 2022; 5-frame temporal context, ResNet-101) against the deployed
single-frame [eWaSR](https://github.com/tersekmatija/eWaSR) (`ewasr_resnet18`)
on BizzyBoat OAK footage from days the deployment logs record segmentation
false positives. Evidence for the option choice in
[rolker/unh_marine_perception#42](https://github.com/rolker/unh_marine_perception/issues/42);
work tracked in [#49](https://github.com/rolker/unh_marine_perception/issues/49).

Findings-only. Nothing here changes `sea_surface_segmentation` or any config.

## Layout

```
benchmarks/temporal_segmentation/
├── README.md
├── requirements.txt      # deps for the separate venv (see file header)
├── extract_window.py     # camera bag + camera + time window -> frames.npz / masks.npz / index.csv
├── run_models.py         # eWaSR ONNX (offline) + WaSR-T over frames.npz -> probs_*.npz
├── metrics.py            # A/B/C statistics -> metrics.csv, timeseries.csv
├── make_overlay.py       # 4-panel overlay video for eyeball review
├── contact_sheet.py      # frames with the most interior detections as RGB | A | B | C (JPEG)
├── reflex_replay.py      # bag TF + camera_info -> water plane -> stop/slowdown boxes: would it have fired?
├── collect_results.sh    # metrics + sheets + overlays for every window; aggregates results/*.csv
└── results/              # committed: summary tables, reflex replay, contact sheets, README
```

## Data

Camera video exists only in the `bizzy_images` camera bags
(`/mnt/nadata/map2026asv/logs/gabby/logs/bizzy_images/bag_*_ffmpeg_seg`); the
main `bizzyboat` bags carry no camera topics. Each camera bag holds, for all
four OAK cameras at 5 fps: `image_raw/ffmpeg` (H.265, 1920x1080), the
on-camera eWaSR softmax mask (`segmentation`, 128x96 `rgb8`, R=obstacle
G=water B=sky, see `sea_surface_segmentation.cpp`), `segmentation/camera_info`,
`/tf` and the local costmap. Bag directory names are local time.

Bitrate matters: all four cameras recorded at 800 kbps from 2026-04-24 through
2026-08-05, then 100 kbps (`unh_echoboats_project11` commits `6181dca`,
`9a54fc1`). Only pre-08-05 bags are representative input.

## Three sources, same frames

| | source | input | what the difference measures |
|---|---|---|---|
| A | recorded on-camera eWaSR | uncompressed 1280x720 preview, stretched to 512x384 on the Myriad X | — (the deployed truth) |
| B | eWaSR ONNX offline | decoded H.265 frame, stretched to 512x384 | A vs B = H.265 compression effect |
| C | WaSR-T (`wasrt_mastr1478.pth`, hist_len 5) | same as B | B vs C = model effect on equal input |

Both models get ImageNet mean/std normalised RGB, which is what the on-camera
blob conversion (`scripts/convert_model.py`) bakes in and what WaSR-T's
`PytorchHubNormalization` does. WaSR-T runs in sequential (stateful) mode with
state cleared once per window. Its native decoder output is 96x128, the same
geometry as the recorded mask, so no resampling is involved in the comparison.

## Running

```bash
V=~/data/neural_nets/wasr_t/.venv/bin/python
B=/mnt/nadata/map2026asv/logs/gabby/logs/bizzy_images
O=~/data/logs/analysis/temporal_segmentation_49

# 1. extract a window (workspace .venv is enough for this step: av + mcap)
.venv/bin/python benchmarks/temporal_segmentation/extract_window.py \
  --bag $B/bag_2026-06-03T14.58.57_ffmpeg_seg --camera oak_starboard \
  --start 2026-06-03T15:50:30 --end 2026-06-03T15:54:00 \
  --out $O/0603_glare_offaxis/oak_starboard

# 2. run both models (GPU; ~120 fps eWaSR, ~6.6 fps WaSR-T fp32 on an RTX 2080 Super)
$V benchmarks/temporal_segmentation/run_models.py --window $O/0603_glare_offaxis/oak_starboard

# 3. numbers + video
$V benchmarks/temporal_segmentation/metrics.py      --window $O/0603_glare_offaxis/oak_starboard
$V benchmarks/temporal_segmentation/make_overlay.py --window $O/0603_glare_offaxis/oak_starboard

# 4. would the reflex have fired? (needs the bag again for /tf + camera_info;
#    --obstacle-prob-min as deployed that day: 0.0 before 2026-06-09, 0.60 after)
$V benchmarks/temporal_segmentation/reflex_replay.py --window $O/0603_glare_offaxis/oak_starboard \
  --bag $B/bag_2026-06-03T14.58.57_ffmpeg_seg --camera oak_starboard --obstacle-prob-min 0.0

# everything at once (steps 3 + sheets, aggregates into results/)
benchmarks/temporal_segmentation/collect_results.sh
```

Model files: `~/data/neural_nets/ewasr_resnet18.onnx` (the deployed model's
source ONNX) and `~/data/neural_nets/wasr_t/{wasrt_mastr1478.pth,repo/}`
(release weights + a clone of the upstream repo, imported as a library).

## Metrics (no ground truth)

There are no labels for this footage, so `metrics.py` reports proxies for the
behaviour that hurts the boat, all computed in the per-frame water band (rows
below the horizon found from source A's sky class, plus a margin, so the
shoreline on the horizon is mostly excluded):

- `obst_frac_water[rule]` — obstacle pixel fraction under the deployed argmax
  rule and under the `obstacle_prob_min` thresholds used in the field
  (0.7 / 0.8 / 0.95).
- `flicker_water` — fraction of pixels whose class changes frame to frame.
- `blobs_water` — connected obstacle components per frame (each is a
  potential reflex point).
- `persist_3of4_water` — share of obstacle pixels that were obstacle in ≥3 of
  the last 4 frames (what a persistence gate keeps; low = glint-like).

`timeseries.csv` gives per-frame counts to locate the moments the logs
describe; `make_overlay.py` renders RGB | A | B | C with obstacle tinted red.

`reflex_replay.py` is the operational yardstick: it reproduces what
`segments_to_pointcloud_reflex` + nav2 Collision Monitor do on the boat
(pixel -> undistorted ray -> `bizzy/base_link_level` via the bag's TF -> water
plane -> stop 5x4 m / slowdown 20x6 m boxes with their `min_points`) and
reports, per source, the frames and episodes that would have triggered a
stop or slowdown. One camera at a time; the boat merges four.

## Results

See [`results/README.md`](results/README.md) and the findings comment on #42.
