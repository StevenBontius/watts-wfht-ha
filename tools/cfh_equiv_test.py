#!/usr/bin/env python3
"""
Cross-language equivalence test for the call-for-heat P-loop.

The firmware C++ loop (firmware/src/cfh.h) is a port of this repo's reference
emulator (tools/wfht_emulator.py). This harness drives BOTH with one scripted
(time, setpoint, ambient) scenario and asserts they emit an identical byte-20
flag at every sample -- proving the port has not drifted from the reference.

    python tools/cfh_equiv_test.py     # exits 0 if identical, 1 on any mismatch

It builds firmware/test/cfh_runner.cpp (the real cfh.h over stdin/stdout) with
g++ and pipes the scenario through it, comparing against the Python Emulator.

Two deliberate alignments make the comparison fair:
  * Bp = 2.1 -- the emulator defaults to 2.0, but the firmware's effective
    proportional band is 2.1 (see CLAUDE.md / CFH_BP_C), so we pass Params(bp=2.1).
  * 1-second sample grid -- the firmware uses float32 + integer milliseconds
    while the emulator uses float64 + datetime; sampling on whole seconds keeps
    every sample clear of the sub-millisecond rounding around pulse boundaries.
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
from datetime import datetime, timedelta
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from wfht_emulator import Emulator, Params  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
RUNNER_SRC = REPO / "firmware" / "test" / "cfh_runner.cpp"
BP = 2.1  # match firmware CFH_BP_C (emulator default is 2.0)

# Scripted scenario: list of (duration_s, setpoint, ambient) segments, sampled
# at 1 s. Chosen to exercise every branch of the loop and to cross cycle
# boundaries (free-running Cy=900 s from t=0):
SEGMENTS = [
    # steady call -- ON clamps to Cy-Of_min (780 s); crosses the 900 s boundary
    (1000, 22.0, 20.0),
    # SP drop makes error negative mid-pulse -> immediate cut, then steady idle
    (300, 20.0, 22.0),
    # demand onset from idle -> out-of-cycle On_min pulse, then boundary-adopted duty
    (1000, 23.0, 22.0),
    # off zone (sp 0 on the wire) -> never calls
    (100, 0.0, 22.0),
    # hard demand onset (duty clips to 1) mid-cycle -> On_min pulse, then full window
    (600, 25.0, 20.0),
]


def build_series() -> list[tuple[int, float, float]]:
    series, t = [], 0
    for dur, sp, amb in SEGMENTS:
        for _ in range(dur):
            series.append((t, sp, amb))
            t += 1
    return series


def run_python(series) -> list[int]:
    emu = Emulator(Params(bp=BP))
    base = datetime(2025, 1, 1)
    return [emu.update(base + timedelta(seconds=t), sp, amb) for t, sp, amb in series]


def run_cpp(series) -> list[int]:
    with tempfile.TemporaryDirectory() as d:
        binpath = Path(d) / "cfh_runner"
        subprocess.run(
            ["g++", "-std=c++17", "-O2", str(RUNNER_SRC), "-o", str(binpath)],
            check=True,
        )
        stdin = "".join(f"{t*1000} {sp} {amb}\n" for t, sp, amb in series)
        out = subprocess.run(
            [str(binpath)], input=stdin, capture_output=True, text=True, check=True
        )
    return [int(x) for x in out.stdout.split()]


def main() -> int:
    series = build_series()
    py = run_python(series)
    cpp = run_cpp(series)

    if len(py) != len(cpp):
        print(f"length mismatch: python={len(py)} cpp={len(cpp)}")
        return 1

    mismatches = [
        (t, sp, amb, p, c)
        for (t, sp, amb), p, c in zip(series, py, cpp)
        if p != c
    ]
    if mismatches:
        print(f"{len(mismatches)} mismatch(es) over {len(series)} samples:")
        for t, sp, amb, p, c in mismatches[:20]:
            print(f"  t={t:>4}s sp={sp} amb={amb}  python=0x{p:02x}  cpp=0x{c:02x}")
        return 1

    # Sanity: the scenario must actually exercise both states, or "all idle"
    # would pass vacuously.
    assert any(py), "scenario never called for heat -- not a real test"
    assert not all(py), "scenario never went idle -- not a real test"
    print(f"OK: {len(series)} samples identical (calls: {sum(1 for f in py if f)} / {len(py)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
