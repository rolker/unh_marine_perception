# Baseline: bizzy production `udp_bridge` load 2026-04-14

Production `udp_bridge` telemetry from bag `~/data/logs/bizzyboat/bag_2026-04-14T12.43.44`
— a running bizzyboat deployment with the current (JPEG-throttled) transport
stack. Complements `bizzy_images_2026-04-21.md`, which measures **benchmark
source bags** (raw JPEG at full 5 Hz); this file measures **what the wire
actually carries** after `udp_bridge` rate-throttling.

Derived from `TopicStatisticsArray` and `BridgeInfo` in the bag, not from
the camera `CompressedImage` topics directly.

## Traffic families on vpn (success path)

| Traffic family | Mbps |
|---|---:|
| oak_forward image_raw/compressed | 1.062 |
| oak_forward segmentation | 0.087 |
| mavros (imu, gps, battery) | 0.080 |
| /tf | 0.051 |
| /diagnostics | 0.024 |
| /bizzy/odom | 0.018 |
| mission / camera_info / misc | ~0.030 |
| **Total vpn (success)** | **1.36 Mbps of 8 Mbps cap** |

Non-camera telemetry is only **~0.28 Mbps** — plenty of vpn headroom.

## Network topology clarification

The router handles **LTE ↔ Starlink failover transparently** and presents a
single pipe to `udp_bridge` on the `vpn` connection. The `vpn`-vs-`wifi`
duplication visible in the bridge stats is **LOS (wifi) vs OTH (vpn)**
redundancy, not LTE/Starlink double-paying. So cost math for OTH is just
"one pipe, whichever link the router picked."

## Cost-aware operating points for all four cameras on vpn

Projection from the [extended benchmark matrix](../results/matrix_20260421_232314.md)
extrapolated to four cameras. Non-video overhead: 4× segmentation 0.35 +
4× camera_info 0.04 + existing telemetry 0.28 = **0.67 Mbps**.
`oak_aft` is extrapolated from the other 5 Hz cameras at each target (its
mid-band cells hit a PyAV decoder anomaly — see `results/`).

| Target | 4-cam video | + non-video | Total vpn | % cap | GB / 6h mission | SSIM |
|---|---:|---:|---:|---:|---:|---:|
| **b2500k g30** | 0.47 | +0.67 | 1.14 | 14% | 3.1 | 0.77 |
| **b4000k g30** | 0.72 | +0.67 | 1.39 | 17% | 3.8 | 0.82 |
| b8000k g30 | 1.45 | +0.67 | 2.12 | 27% | 5.7 | 0.88 |
| b12000k g30 | 2.13 | +0.67 | 2.80 | 35% | 7.6 | 0.91 |
| b17000k g30 | 2.95 | +0.67 | 3.62 | 45% | 9.8 | 0.93 |
| b20000k g30 | 3.43 | +0.67 | 4.10 | 51% | 11.1 | 0.94 |

The **b4000k target ≈ today's single-camera vpn usage** (1.062 Mbps for
oak_forward), spread across all four at 5 Hz — i.e., **same budget, 4× the
cameras, 10× the frame rate.**

## Production default recommendation

**`b4000k / GOP 30`.** Four cameras + segmentation at ~1.4 Mbps total,
~3.8 GB per 6-hour mission, SSIM 0.82 — "good, visible compression
artifacts but not distracting". Fits on degraded LTE, keeps Starlink
overage small, still good enough for situational awareness at 10× today's
frame rate.

This operating point is the default `h265_bitrate_kbps` proposed in the
follow-up implementation issue
[#4](https://github.com/rolker/unh_marine_perception/issues/4)
(on-device `VideoEncoder` integration in `depthai_marine`). The
calibration run there will tell us whether the Myriad X encoder hits its
target precisely enough to let us drop the default even lower.
