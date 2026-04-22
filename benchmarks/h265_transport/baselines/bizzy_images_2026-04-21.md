# Baseline: bizzy_images 2026-04-21

JPEG (`CompressedImage`) bandwidth from three bizzyboat bags recorded 2026-04-21,
stored under `~/data/logs/bizzy_images/` on the dev host. Four OAK cameras at
1280x720 at a nominal 5 Hz (3.5 Hz for `oak_starboard`). ~2 minutes each.

Generated with `measure_bag_bw.py` — regenerate with:

```bash
for b in ~/data/logs/bizzy_images/bag_2026-04-21T*; do
  echo "## $(basename $b)"
  .venv/bin/python3 benchmarks/h265_transport/measure_bag_bw.py "$b" \
    --topic-regex '(image_raw/compressed|segmentation/compressed)$' --markdown
done
```

## bag_2026-04-21T13.58.31 (119.57 s)

| Topic | Type | Count | Rate (Hz) | Avg (KB) | BW (KB/s) | BW (Mbps) |
|---|---|---:|---:|---:|---:|---:|
| `/bizzy/sensors/cameras/oak_aft/image_raw/compressed` | `CompressedImage` | 598 | 5.00 | 247.43 | 1237.5 | 9.67 |
| `/bizzy/sensors/cameras/oak_forward/image_raw/compressed` | `CompressedImage` | 598 | 5.00 | 219.75 | 1099.0 | 8.59 |
| `/bizzy/sensors/cameras/oak_port/image_raw/compressed` | `CompressedImage` | 598 | 5.00 | 219.16 | 1096.1 | 8.56 |
| `/bizzy/sensors/cameras/oak_starboard/image_raw/compressed` | `CompressedImage` | 421 | 3.52 | 214.45 | 755.1 | 5.90 |
| `/bizzy/sensors/cameras/oak_port/segmentation/compressed` | `CompressedImage` | 598 | 5.00 | 3.12 | 15.6 | 0.12 |
| `/bizzy/sensors/cameras/oak_aft/segmentation/compressed` | `CompressedImage` | 598 | 5.00 | 3.07 | 15.4 | 0.12 |
| `/bizzy/sensors/cameras/oak_forward/segmentation/compressed` | `CompressedImage` | 598 | 5.00 | 2.89 | 14.4 | 0.11 |
| `/bizzy/sensors/cameras/oak_starboard/segmentation/compressed` | `CompressedImage` | 420 | 3.51 | 2.71 | 9.5 | 0.07 |

**Total (all 8 camera topics): 4242.6 KB/s ≈ 33.14 Mbps**

## bag_2026-04-21T14.22.36 (119.50 s)

| Topic | Type | Count | Rate (Hz) | Avg (KB) | BW (KB/s) | BW (Mbps) |
|---|---|---:|---:|---:|---:|---:|
| `/bizzy/sensors/cameras/oak_aft/image_raw/compressed` | `CompressedImage` | 598 | 5.00 | 272.46 | 1363.5 | 10.65 |
| `/bizzy/sensors/cameras/oak_forward/image_raw/compressed` | `CompressedImage` | 597 | 5.00 | 249.62 | 1247.2 | 9.74 |
| `/bizzy/sensors/cameras/oak_port/image_raw/compressed` | `CompressedImage` | 598 | 5.00 | 247.79 | 1240.1 | 9.69 |
| `/bizzy/sensors/cameras/oak_starboard/image_raw/compressed` | `CompressedImage` | 420 | 3.51 | 253.97 | 892.7 | 6.97 |
| `/bizzy/sensors/cameras/oak_aft/segmentation/compressed` | `CompressedImage` | 598 | 5.00 | 2.93 | 14.7 | 0.11 |
| `/bizzy/sensors/cameras/oak_forward/segmentation/compressed` | `CompressedImage` | 598 | 5.00 | 2.92 | 14.6 | 0.11 |
| `/bizzy/sensors/cameras/oak_port/segmentation/compressed` | `CompressedImage` | 597 | 5.00 | 2.77 | 13.8 | 0.11 |
| `/bizzy/sensors/cameras/oak_starboard/segmentation/compressed` | `CompressedImage` | 420 | 3.51 | 2.72 | 9.6 | 0.07 |

**Total (all 8 camera topics): 4796.2 KB/s ≈ 37.47 Mbps**

## bag_2026-04-21T14.35.51 (119.65 s)

| Topic | Type | Count | Rate (Hz) | Avg (KB) | BW (KB/s) | BW (Mbps) |
|---|---|---:|---:|---:|---:|---:|
| `/bizzy/sensors/cameras/oak_forward/image_raw/compressed` | `CompressedImage` | 597 | 5.00 | 299.95 | 1498.4 | 11.71 |
| `/bizzy/sensors/cameras/oak_aft/image_raw/compressed` | `CompressedImage` | 598 | 5.00 | 290.29 | 1452.7 | 11.35 |
| `/bizzy/sensors/cameras/oak_port/image_raw/compressed` | `CompressedImage` | 598 | 5.00 | 272.29 | 1362.6 | 10.64 |
| `/bizzy/sensors/cameras/oak_starboard/image_raw/compressed` | `CompressedImage` | 421 | 3.52 | 288.68 | 1017.0 | 7.95 |
| `/bizzy/sensors/cameras/oak_forward/segmentation/compressed` | `CompressedImage` | 598 | 5.00 | 3.67 | 18.4 | 0.14 |
| `/bizzy/sensors/cameras/oak_port/segmentation/compressed` | `CompressedImage` | 597 | 5.00 | 3.32 | 16.6 | 0.13 |
| `/bizzy/sensors/cameras/oak_aft/segmentation/compressed` | `CompressedImage` | 598 | 5.00 | 3.10 | 15.5 | 0.12 |
| `/bizzy/sensors/cameras/oak_starboard/segmentation/compressed` | `CompressedImage` | 421 | 3.52 | 3.41 | 12.0 | 0.09 |

**Total (all 8 camera topics): 5393.1 KB/s ≈ 42.13 Mbps**

## Summary

Across the three bags:

- Per-camera passthrough JPEG ranges **~6–12 Mbps** depending on scene content
  (rate goes up as scenes get more detailed / darker — compare `oak_forward`:
  8.59 → 9.74 → 11.71 Mbps across the three recordings).
- **Four-camera aggregate JPEG passthrough: 33–42 Mbps** depending on scene.
- Segmentation masks are **0.07–0.14 Mbps per camera** — rounding error on the
  bandwidth budget. Out of scope for compression work.
- Frame size (KB) = bandwidth driver; frame rate is stable.

## Target for H.265

Per `docs.luxonis.com/software/depthai-components/nodes/video_encoder/`, the
OAK hardware VideoEncoder's default bitrate for **720p @ 30 fps** is **4 Mbps**.
The bags run at **5 fps**, so a rate-proportional scaling puts the equivalent
default at roughly **670 kbps per camera** — about **2.7 Mbps for all four**.

That would be a **~12x reduction** from the highest-bandwidth bag measured
here, if visual quality holds. The test matrix in issue #2 brackets this
target with bitrates from 500 kbps to 4 Mbps per camera.
