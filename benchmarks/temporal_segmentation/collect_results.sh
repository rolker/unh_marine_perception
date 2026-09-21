#!/usr/bin/env bash
# Run metrics + contact sheet + overlay for every window under $1 (default
# ~/data/logs/analysis/temporal_segmentation_49) and aggregate metrics.csv
# files into results/summary.csv (and any reflex.csv into results/reflex_summary.csv)
# next to this script.
# Part of rolker/unh_marine_perception#49.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${1:-$HOME/data/logs/analysis/temporal_segmentation_49}"
V="${WASRT_PYTHON:-$HOME/data/neural_nets/wasr_t/.venv/bin/python}"
RES="$HERE/results"
mkdir -p "$RES"
SUMMARY="$RES/summary.csv"
REFLEX="$RES/reflex_summary.csv"
: > "$REFLEX"
first=1
for d in "$ROOT"/*/oak_*; do
  [ -f "$d/probs_wasrt_h5.npz" ] || { echo "skip (no WaSR-T output): $d"; continue; }
  name="$(basename "$(dirname "$d")")_$(basename "$d")"
  echo "== $name"
  "$V" "$HERE/metrics.py" --window "$d"
  "$V" "$HERE/contact_sheet.py" --window "$d" --out "$RES/${name}.jpg"
  [ -f "$d/overlay.mp4" ] || "$V" "$HERE/make_overlay.py" --window "$d"
  if [ $first -eq 1 ]; then
    { printf 'window,'; head -1 "$d/metrics.csv"; } > "$SUMMARY"; first=0
  fi
  tail -n +2 "$d/metrics.csv" | sed "s/^/$name,/" >> "$SUMMARY"
  if [ -f "$d/reflex.csv" ]; then   # written by reflex_replay.py (needs the bag, so not run here)
    [ -s "$REFLEX" ] || { printf 'window,'; head -1 "$d/reflex.csv"; } > "$REFLEX"
    tail -n +2 "$d/reflex.csv" | sed "s/^/$name,/" >> "$REFLEX"
  fi
done
echo "summary: $SUMMARY"
echo "reflex:  $REFLEX"
