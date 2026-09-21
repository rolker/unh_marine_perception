#!/usr/bin/env python3
"""Run eWaSR (offline, ONNX) and WaSR-T (temporal, PyTorch) over an extracted
window and store per-frame 3-class softmax maps at the recorded mask geometry.

Inputs:  <window>/frames.npz from extract_window.py  (N,384,512,3 uint8 RGB)
Outputs: <window>/probs_ewasr_offline.npz   probs float16 (N,96,128,3) [obstacle, water, sky] + stamp_ns
         <window>/probs_wasrt_h<H>.npz       probs float16 (N,96,128,3) + stamp_ns

Both models get identical input: the 512x384 stretched RGB frame, ImageNet
mean/std normalisation. That matches the on-camera eWaSR blob conversion
(convert_model.py: --mean_values/--scale_values + --reverse_input_channels on a
BGR888p ImageManip output => RGB in ImageNet stats) and WaSR-T's
PytorchHubNormalization. WaSR-T runs in sequential (stateful) mode, state
cleared once at the start of the window, so frame i sees frames i-H..i-1.

Part of rolker/unh_marine_perception#49 (evidence for #42).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

SEG_W, SEG_H = 128, 96
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def normalize(frame_u8: np.ndarray) -> np.ndarray:
    """(H,W,3) uint8 RGB -> (1,3,H,W) float32 ImageNet-normalised."""
    x = frame_u8.astype(np.float32) / 255.0
    x = (x - MEAN) / STD
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None])


def softmax_c(logits: np.ndarray) -> np.ndarray:
    """(3,H,W) logits -> (H,W,3) probabilities."""
    z = logits - logits.max(axis=0, keepdims=True)
    e = np.exp(z)
    return (e / e.sum(axis=0, keepdims=True)).transpose(1, 2, 0)


def run_ewasr(frames: np.ndarray, onnx_path: str) -> np.ndarray:
    import torch  # noqa: F401  -- imported first so the pip-installed CUDA 12 /
    # cuDNN 9 shared libraries are loaded; onnxruntime-gpu (CUDA 12 build, see
    # requirements.txt) finds them through the process instead of the system
    # and silently falls back to the CPU provider (~1 fps) otherwise.
    import onnxruntime as ort

    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    sess = ort.InferenceSession(onnx_path, providers=providers)
    print(f"eWaSR ONNX providers: {sess.get_providers()}")
    inp = sess.get_inputs()[0].name
    out = np.zeros((len(frames), SEG_H, SEG_W, 3), dtype=np.float16)
    t = time.time()
    for i, f in enumerate(frames):
        logits = sess.run(["prediction"], {inp: normalize(f)})[0][0]  # (3,96,128)
        out[i] = softmax_c(logits)
    print(f"eWaSR offline: {len(frames)} frames in {time.time() - t:.1f}s")
    return out


def run_wasrt(frames: np.ndarray, weights: str, repo: str, hist_len: int, fp16: bool) -> np.ndarray:
    sys.path.insert(0, repo)
    import torch
    import torch.nn.functional as F
    import pytorch_lightning.loggers as pl_loggers

    # The upstream repo (last pushed 2023-11, pins pytorch-lightning 1.4) defines a
    # training-only logger subclass of `LoggerCollection` at import time in
    # wasr_t/utils.py, and wasr_t/wasr_t.py imports that module. Lightning 2.x
    # removed the class. Alias it so the import succeeds; nothing in inference
    # touches it. The alias is process-global for the interpreter's lifetime,
    # which is acceptable in a standalone script. Kept here rather than patching
    # the upstream checkout so the repo stays a pristine clone at a known commit
    # (1b5360af2040, see README).
    for name in ("LoggerCollection", "LightningLoggerBase"):
        if not hasattr(pl_loggers, name):
            setattr(pl_loggers, name, pl_loggers.Logger)
    from wasr_t.wasr_t import wasr_temporal_resnet101

    def load_weights(path: str) -> dict:
        sd = torch.load(path, map_location="cpu", weights_only=False)
        return sd["model"] if "model" in sd else sd

    dev = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = wasr_temporal_resnet101(pretrained=False, hist_len=hist_len)
    model.load_state_dict(load_weights(weights))
    model.sequential()
    model.clear_state()
    model = model.eval().to(dev)
    if fp16:
        model = model.half()
    print(f"WaSR-T RN101 hist_len={hist_len} on {dev} fp16={fp16}")

    out = np.zeros((len(frames), SEG_H, SEG_W, 3), dtype=np.float16)
    t = time.time()
    with torch.no_grad():
        for i, f in enumerate(frames):
            x = torch.from_numpy(normalize(f)).to(dev)
            if fp16:
                x = x.half()
            logits = model({"image": x})["out"]  # (1,3,h,w) at decoder resolution
            if i == 0:
                print(f"WaSR-T native output {tuple(logits.shape[-2:])} -> {SEG_H}x{SEG_W}")
            logits = F.interpolate(logits.float(), size=(SEG_H, SEG_W), mode="bilinear", align_corners=False)
            out[i] = softmax_c(logits[0].cpu().numpy())
    print(f"WaSR-T: {len(frames)} frames in {time.time() - t:.1f}s ({len(frames) / (time.time() - t):.1f} fps)")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--window", required=True, help="directory holding frames.npz")
    ap.add_argument("--ewasr-onnx", default=str(Path.home() / "data/neural_nets/ewasr_resnet18.onnx"))
    ap.add_argument("--wasrt-weights", default=str(Path.home() / "data/neural_nets/wasr_t/wasrt_mastr1478.pth"))
    ap.add_argument("--wasrt-repo", default=str(Path.home() / "data/neural_nets/wasr_t/repo"))
    ap.add_argument("--hist-len", type=int, default=5)
    ap.add_argument("--fp16", action="store_true")
    ap.add_argument("--skip-ewasr", action="store_true")
    ap.add_argument("--skip-wasrt", action="store_true")
    args = ap.parse_args()

    win = Path(args.window)
    fr = np.load(win / "frames.npz")
    frames, stamps = fr["frames"], fr["stamp_ns"]
    print(f"{win}: {len(frames)} frames {frames.shape[1:]}")

    # stamp_ns travels with every per-frame output so the consumers (metrics,
    # reflex_replay, overlays) can refuse a stale file after a re-extraction.
    if not args.skip_ewasr:
        p = run_ewasr(frames, args.ewasr_onnx)
        np.savez_compressed(win / "probs_ewasr_offline.npz", probs=p, stamp_ns=stamps)
    if not args.skip_wasrt:
        p = run_wasrt(frames, args.wasrt_weights, args.wasrt_repo, args.hist_len, args.fp16)
        np.savez_compressed(win / f"probs_wasrt_h{args.hist_len}.npz", probs=p, stamp_ns=stamps)
    return 0


if __name__ == "__main__":
    sys.exit(main())
