# Results — WaSR-T vs eWaSR on 2026 BizzyBoat footage

Run 2026-09-21 on the dev host (RTX 2080 Super). Windows chosen from the
deployment logs (`unh_echoboats_project11/docs/logs/2026/`) where the operator
reported segmentation false positives, matched to the `bizzy_images` camera
bags on the NAS archive. All windows are 800 kbps H.265, pre-2026-08-05.

Sources: **A** recorded on-camera eWaSR (uncompressed input), **B** eWaSR ONNX
on the decoded H.265 frame, **C** WaSR-T (`wasrt_mastr1478.pth`, hist_len 5)
on the same frame. A vs B = compression effect; B vs C = model effect.

## Would the reflex have fired? (`reflex_summary.csv`)

`reflex_replay.py` projects each source's obstacle pixels through the bag's
own camera_info and TF onto the water plane in `bizzy/base_link_level` and
applies the deployed Collision Monitor boxes: **stop** 5 m × 4 m ahead,
≥ 5 points; **slowdown** 20 m × 6 m, ≥ 4 points. Confidence floor as deployed
that day: none until `unh_echoboats_project11` commit `4ce5086` (2026-06-13,
"activate reflex confidence floor on BizzyBoat") set `obstacle_prob_min`
0.60 on the reflex node; the June 3 and May 28 windows predate it. Single camera per row — the
boat merges four, so these are lower bounds on what it saw.

| window | condition | source | stop frames (episodes) | slowdown frames (episodes) |
|---|---|---|---|---|
| 06-03 15:50 stbd, 1050 fr | sun glitter (confirmed FPs) | A | 6 (6) | 13 (13) |
| | | B | 28 (18) | 96 (67) |
| | | **C** | **0** | **0** |
| 06-03 15:50 fwd | same leg | A | 10 (9) | 26 (21) |
| | | B | 12 (9) | 38 (33) |
| | | **C** | **0** | **0** |
| 06-03 15:50 port | same leg | A | 3 (2) | 12 (9) |
| | | B | 7 (5) | 37 (22) |
| | | **C** | **0** | **0** |
| 06-03 16:06 fwd, 1800 fr | heading into the sun | A | 95 (55) | 214 (111) |
| | | B | 177 (94) | 457 (201) |
| | | **C** | **0** | **2 (2)** |
| 06-17 12:37 fwd, 1499 fr | cloud reflections ("surface clutter"); operator zeroed CA zones | A | **754 (160)** | 941 (153) |
| | | B | 621 (200) | 824 (171) |
| | | **C** | **2 (2)** | **4 (4)** |
| 06-22 10:12 fwd, 1500 fr | mirror-calm morning | A | 4 (1) | 9 (5) |
| | | B | 3 (1) | 84 (35) |
| | | **C** | **0** | **0** |
| 06-25 10:40 fwd, 2701 fr | sky reflection bending lines; CA disabled 35 min | A | 186 (105) | 338 (158) |
| | | B | 447 (176) | 634 (180) |
| | | **C** | **0** | **1 (1)** |
| 06-22 15:10 fwd | overcast ripple, no incident | A / B / C | 0 | 0 |
| **05-28 13:44 fwd, 1500 fr** | **control: real buoys at the pier** | A | 85 (37) | 446 (91) |
| | | B | 99 (43) | 469 (78) |
| | | **C** | **46 (1)** | **386 (24)** |

Reading: in every false-positive window the recorded model would have
stopped the boat repeatedly (June 17: half of all frames, 160 separate stop
episodes in five minutes) and WaSR-T would have stopped it 0–2 frames. On
the buoy control WaSR-T still stops, as **one sustained episode** where the
recorded model produced 37 flickering ones.

## Interior detections (`summary.csv`)

Obstacle blobs floating in open water (not touching the horizon strip), argmax
rule: blobs per frame · share of frames with ≥ 1.

| window | A recorded | B eWaSR on H.265 | C WaSR-T |
|---|---|---|---|
| 06-03 15:50 stbd | 1.09 · 62% | 3.00 · 85% | 0.20 · 16% |
| 06-03 15:50 fwd | 0.48 · 34% | 0.49 · 34% | 0.08 · 7% |
| 06-03 15:50 port | 0.46 · 31% | 0.56 · 32% | 0.25 · 21% |
| 06-03 16:06 fwd | 2.03 · 70% | 3.70 · 90% | 0.11 · 8% |
| 06-17 12:37 fwd | 7.90 · 87% | 4.78 · 73% | 0.02 · 1.5% |
| 06-22 10:12 fwd | 4.52 · 95% | 6.76 · 97% | 0.06 · 4.5% |
| 06-25 10:40 fwd | 2.13 · 60% | 2.51 · 58% | 0.004 · 0.4% |
| 06-22 15:10 fwd | 0.05 · 5% | 0.01 · 0.5% | 0.00 · 0% |
| 05-28 13:44 fwd (control) | 1.41 · 70% | 1.13 · 66% | 0.65 · 49% |

## Caveats

- **No ground truth.** The contact sheets (`*.jpg`, one per window: RGB | A |
  B | C, obstacle tinted red) are the evidence that the interior detections are
  glitter / cloud / sky reflections and not objects. Reviewed by eye.
- **H.265 input.** WaSR-T saw compressed frames; the on-camera model saw raw
  ones. Compression *hurts* eWaSR (B ≫ A in most windows), so if anything the
  comparison is tilted against the temporal model. It also says host-side
  inference on the bridged video (option C in #42) would be worse than what
  is deployed.
- **Shoreline thickening.** WaSR-T labels the treeline's mirror image as
  obstacle on glassy water (06-25 sheet, rows 5–6), pushing the shore band
  toward the boat. Not a reflex trigger in any window, but a costmap effect
  to expect.
- **Small far targets.** On the buoy control WaSR-T's frame share is lower
  (49% vs 70%): the near buoy is held solidly, the far small ones are not
  always. Pixel count per frame is on par with B.
- **Frame rate.** 5 fps footage vs ~10 fps training video; hist_len 5 spans
  1 s here. Not probed further; the effect is evidently sufficient at 5 fps.
- **Missing conditions.** No camera bag exists for the 07-29 rain-drop
  incident, the 06-05 / 06-18 whitecap incidents, or the 06-09 / 06-16 reflex
  false-stop sessions. Rain and whitecaps remain untested.
- **05-21 "collision control" excluded**: the forward camera was already
  knocked off its mount and looking at the hull for the whole window.
- Reflex replay is per camera; the deployed Collision Monitor sums all four.
  Every frame in every window had a /tf sample within 38 ms (median), so the
  nearest-sample pose approximation and the frame denominators are clean
  (`frames_evaluated == frames`, `tf_missing == 0` in `reflex_summary.csv`).
  Config values are the tracked ones for each date; the 06-09 live shrink of
  the stop box (5×4 → 3×3 m) noted in the log is not in git and is not applied.
