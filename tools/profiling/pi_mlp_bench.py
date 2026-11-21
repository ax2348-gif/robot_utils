#!/usr/bin/env python3
"""How big a neural "system 0" the Pi can run per control tick (on the Pi, robot venv): pi_mlp_bench.py [MODE] [SECONDS]

MODE "burst" (default): per network, 2000 back-to-back single-sample forward passes after a warm-up; reports
p50/p99/max microseconds for numpy (OpenBLAS) and PyTorch, single thread (a control loop must not take other cores).
MODE "paced": per MLP from 0.2M parameters up, a 100 Hz loop for SECONDS (default 20) in numpy (the faster one here):
the per-step time and any step that overran its 10 ms slot, i.e. what a real system-0 process would see beside the
control loops (run it under load and watch the arm's overruns).
Networks span the literature: tiny quadrotor controllers (~3k params) to Helix-S0-sized (~10M) MLPs and transformers.
Batch 1, float32, random weights (only the cost matters).
"""
import json, os, sys, time

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import numpy as np  # noqa: E402
import torch  # noqa: E402

torch.set_num_threads(1)
MODE = sys.argv[1] if len(sys.argv) > 1 else "burst"
SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
MLPS = {"tiny 32-64-32-8": [32, 64, 32, 8],
        "50k 64-256-128-12": [64, 256, 128, 12],
        "200k 64-512-256-128-12": [64, 512, 256, 128, 12],
        "1M 64-1024-512-512-12": [64, 1024, 512, 512, 12],
        "2M 64-1024x3-12": [64, 1024, 1024, 1024, 12],
        "13M 64-2048x4-12": [64, 2048, 2048, 2048, 2048, 12]}
TRANSFORMERS = {"3M transformer d256 L4 16tok": (256, 4, 16), "10M transformer d512 L3 16tok": (512, 3, 16)}


def torch_mlp(sizes):
    layers = []
    for a, b in zip(sizes, sizes[1:]):
        layers += [torch.nn.Linear(a, b), torch.nn.ELU()]
    return torch.nn.Sequential(*layers[:-1]).eval()


def torch_tf(d, n, tok):
    enc = torch.nn.TransformerEncoder(torch.nn.TransformerEncoderLayer(d, 8, 4 * d, batch_first=True), n).eval()
    return enc, torch.randn(1, tok, d)


def params(m):
    return sum(p.numel() for p in m.parameters())


def pct(a, q):
    a = sorted(a)
    return round(a[min(len(a) - 1, int(q / 100 * len(a)))] * 1e6, 1)


def burst(fn, n=2000, warm=200):
    for _ in range(warm):
        fn()
    t = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        t.append(time.perf_counter() - t0)
    return {"p50_us": pct(t, 50), "p99_us": pct(t, 99), "max_us": pct(t, 100)}


def paced(fn, sec):
    for _ in range(100):
        fn()
    t, over, period = [], 0, 0.01
    nxt = time.perf_counter()
    end = nxt + sec
    while nxt < end:
        t0 = time.perf_counter()
        fn()
        t1 = time.perf_counter()
        t.append(t1 - t0)
        nxt += period
        if t1 > nxt:
            over += 1
            nxt = t1
        time.sleep(max(0.0, nxt - time.perf_counter()))
    return {"steps": len(t), "p50_us": pct(t, 50), "p99_us": pct(t, 99), "max_us": pct(t, 100), "overruns": over}


def main():
    out = {}
    with torch.inference_mode():
        for name, sizes in MLPS.items():
            m = torch_mlp(sizes)
            x = torch.randn(1, sizes[0])
            ws = [(l.weight.detach().numpy().T.copy(), l.bias.detach().numpy().copy()) for l in m if isinstance(l, torch.nn.Linear)]
            xn = x.numpy().copy()

            def np_fwd():
                h = xn
                for i, (w, b) in enumerate(ws):
                    h = h @ w + b
                    if i < len(ws) - 1:
                        h = np.where(h > 0, h, np.expm1(np.minimum(h, 0)))
                return h
            r = {"params": params(m)}
            if MODE == "burst":
                r["numpy"] = burst(np_fwd)
                r["torch"] = burst(lambda: m(x))
            else:
                if r["params"] < 150_000:
                    continue
                r["numpy_100hz"] = paced(np_fwd, SEC)
            out[name] = r
            print(name, json.dumps(r), flush=True)
        for name, (d, n, tok) in (TRANSFORMERS.items() if MODE == "burst" else ()):
            m, x = torch_tf(d, n, tok)
            r = {"params": params(m)}
            if MODE == "burst":
                r["torch"] = burst(lambda: m(x), n=500, warm=50)
            else:
                r["torch_100hz"] = paced(lambda: m(x), SEC)
            out[name] = r
            print(name, json.dumps(r), flush=True)
    print(json.dumps(out), flush=True)


if __name__ == "__main__":
    main()
