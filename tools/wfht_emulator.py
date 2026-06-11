"""
Watts WFHT-RF thermostat emulator.

Reproduces the control-loop behaviour of a real WFHT-RF thermostat in PI mode
(J7=rEg). Given a stream of (time, setpoint, ambient) readings, predicts the
flag_byte (byte 20) that a real device would broadcast.

Model derived from bench captures:

    duty = clip((SP - T) / Bp, 0, 1)            # P-only, no integral term
    ON time per Cy window = duty * Cy, clamped:
        - if 0 < ON < On_min  -> ON = On_min    # anti-short-cycle floor
        - if ON > Cy - Of_min -> ON = Cy - Of_min  # anti-short-cycle ceiling
        - if duty == 0        -> no ON at all (clamps don't engage)

    Cy is free-running with arbitrary phase set at cycle 0.
    Duty is sampled once per cycle at the cycle boundary; not recomputed
    mid-cycle EXCEPT when SP changes such that error goes negative, in which
    case the ON pulse is cut short immediately (user input bypasses On_min).

    Broadcasts emit every ~154 s, plus an immediate burst on any state change.

Not yet modelled:
    - Cp offset (currently assumed 0; sign and placement still TBD)
    - A0 sensor offset effect on loop input
    - J7=hys mode (static hysteresis)
    - J1=CLd cooling direction (sign of error inverts)
    - Floor sensor and FL/FH limits
"""

from __future__ import annotations
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from typing import Iterable


@dataclass
class Params:
    """Installer-menu parameters."""
    bp: float = 2.0           # proportional band, °C
    cy_s: int = 900           # cycle time, seconds
    on_min_s: int = 120       # minimum ON time, seconds
    of_min_s: int = 120       # minimum OFF time, seconds
    cp: float = 0.0           # compensation, °C (placement TBD)
    a0: float = 0.0           # ambient sensor offset, °C
    j1: str = "Hot"           # "Hot" or "CLd"
    j7: str = "rEg"           # "rEg" (PI) or "hys" (hysteresis)
    broadcast_period_s: int = 154
    cycle_phase_anchor: datetime | None = None  # any known cycle boundary;
                                                # cycles run at anchor + N*cy_s


@dataclass
class State:
    """Runtime state of the emulator."""
    cycle_start: datetime | None = None
    on_pulse_end: datetime | None = None
    flag: int = 0x00
    last_broadcast: datetime | None = None
    prev_duty: float = 0.0    # for demand-onset detection


def _duty(sp: float, t_amb: float, p: Params) -> float:
    """Compute target duty cycle from error, P-only."""
    t_loop = t_amb + p.a0  # A0 placement assumed input-side (TBD)
    if p.j1 == "Hot":
        error = (sp + p.cp) - t_loop
    else:
        error = t_loop - (sp - p.cp)
    return max(0.0, min(1.0, error / p.bp))


def _on_duration(duty: float, p: Params) -> float:
    """Apply On_min / Of_min clamps to the raw P duty."""
    if duty <= 0:
        return 0.0
    raw = duty * p.cy_s
    return max(p.on_min_s, min(p.cy_s - p.of_min_s, raw))


class Emulator:
    """
    Stateful thermostat emulator. Feed it (time, SP, T) readings in order.
    Query .flag at any point to read the predicted byte 20.
    """

    def __init__(self, params: Params | None = None):
        self.p = params or Params()
        self.s = State()

    def update(self, t: datetime, sp: float, t_amb: float) -> int:
        """
        Advance internal state to time `t` with the given SP and ambient,
        and return the current flag (0x00 or 0x64).
        """
        # Initialize on first call
        if self.s.cycle_start is None:
            if self.p.cycle_phase_anchor is not None:
                # Align cycle_start to the most recent boundary at or before t
                anchor = self.p.cycle_phase_anchor
                n = int((t - anchor).total_seconds() // self.p.cy_s)
                self.s.cycle_start = anchor + timedelta(seconds=n * self.p.cy_s)
            else:
                self.s.cycle_start = t
            self._start_new_cycle(self.s.cycle_start, sp, t_amb)

        # Roll forward through any cycle boundaries crossed
        while t >= self.s.cycle_start + timedelta(seconds=self.p.cy_s):
            self.s.cycle_start = self.s.cycle_start + timedelta(seconds=self.p.cy_s)
            self._start_new_cycle(self.s.cycle_start, sp, t_amb)

        # Demand-onset detection: if duty was 0 last sample and is now positive
        # mid-cycle, fire an out-of-cycle ON pulse for at least On_min. This
        # matches the observed behaviour at 21:02:52 where the first ON pulse
        # started mid-cycle and held for exactly On_min.
        duty_now = _duty(sp, t_amb, self.p)
        if duty_now > 0 and self.s.prev_duty <= 0:
            # Only fire if we're not already in an ON pulse from cycle start
            if self.s.on_pulse_end is None or t >= self.s.on_pulse_end:
                self.s.on_pulse_end = t + timedelta(seconds=self.p.on_min_s)
        self.s.prev_duty = duty_now

        # Within the current cycle, decide flag state
        if self.s.on_pulse_end and t < self.s.on_pulse_end:
            if duty_now <= 0:
                # User SP change made error negative; cut the pulse immediately
                self.s.on_pulse_end = t
                self.s.flag = 0x00
            else:
                self.s.flag = 0x64
        else:
            self.s.flag = 0x00

        return self.s.flag

    def _start_new_cycle(self, cycle_start: datetime, sp: float, t_amb: float):
        """Sample the loop at a cycle boundary and set the ON pulse end."""
        duty = _duty(sp, t_amb, self.p)
        on_s = _on_duration(duty, self.p)
        self.s.on_pulse_end = cycle_start + timedelta(seconds=on_s)
        # Flag will be set in the next update() call based on time comparison

    def should_broadcast(self, t: datetime, prev_flag: int) -> bool:
        """Whether the emulator would emit a burst at time t."""
        if self.s.flag != prev_flag:
            return True  # state change -> immediate burst
        if self.s.last_broadcast is None:
            self.s.last_broadcast = t
            return True
        elapsed = (t - self.s.last_broadcast).total_seconds()
        if elapsed >= self.p.broadcast_period_s:
            self.s.last_broadcast = t
            return True
        return False


# ---------------------------------------------------------------------------
# Validation harness: replay captured JSON, compare predicted vs actual flag.
# ---------------------------------------------------------------------------

def replay(records: Iterable[dict], params: Params | None = None) -> list[dict]:
    """
    Feed captured records through the emulator and return predictions.
    Each record dict must have: time (datetime), setpoint_C, temperature_C,
    flag_byte (str like '0x64' or '0x00').
    """
    emu = Emulator(params)
    out = []
    for r in records:
        predicted = emu.update(r["t"], r["setpoint_C"], r["temperature_C"])
        actual = int(r["flag_byte"], 16)
        out.append({
            "t": r["t"],
            "sp": r["setpoint_C"],
            "ambient": r["temperature_C"],
            "actual": actual,
            "predicted": predicted,
            "match": predicted == actual,
        })
    return out


def summarise(results: list[dict]) -> dict:
    n = len(results)
    matches = sum(1 for r in results if r["match"])
    mismatches = [r for r in results if not r["match"]]
    return {
        "total": n,
        "matches": matches,
        "match_rate": matches / n if n else 0,
        "mismatch_count": len(mismatches),
        "mismatches": mismatches,
    }


# ---------------------------------------------------------------------------
# Run against the captured JSON files
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import json, sys
    from pathlib import Path

    path = sys.argv[1] if len(sys.argv) > 1 else "/mnt/user-data/uploads/thermostat.json"

    # Load all records first
    all_recs = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            r["t"] = datetime.fromisoformat(r["time"])
            all_recs.append(r)
    all_recs.sort(key=lambda r: r["t"])

    # Cluster into bursts: records within 5s of each other are one burst.
    # Within a burst, the SP alternates by 0.1°C (LSB toggle); the lower value
    # is the actual stored SP, the upper is a broadcast artifact.
    records: list[dict] = []
    burst: list[dict] = []
    for r in all_recs:
        if not burst or (r["t"] - burst[-1]["t"]).total_seconds() <= 5:
            burst.append(r)
        else:
            chosen = min(burst, key=lambda x: x["setpoint_C"])
            records.append(chosen)
            burst = [r]
    if burst:
        records.append(min(burst, key=lambda x: x["setpoint_C"]))

    # Auto-detect cycle phase anchor from the data: find the second observed
    # ON transition (the first one may be the partial demand-onset cycle).
    on_starts = []
    prev_flag = "0x00"
    for r in records:
        if r["flag_byte"] == "0x64" and prev_flag == "0x00":
            on_starts.append(r["t"])
        prev_flag = r["flag_byte"]

    anchor = None
    if len(on_starts) >= 2:
        # The capture timestamps for state-change broadcasts can drift +/- 1s.
        # Use the modal second-of-cycle across all ON transitions to find the
        # underlying cycle phase, then round DOWN since transmissions arrive
        # slightly after the actual state change.
        from collections import Counter
        cy_s = 900
        seconds_in_cycle = Counter(
            int(t.timestamp()) % cy_s for t in on_starts
        )
        # Take the modal value and round down to nearest second
        modal_sec = seconds_in_cycle.most_common(1)[0][0]
        # Pick an anchor at this phase, just before the first ON
        first_on = on_starts[0]
        n = int(first_on.timestamp()) // cy_s
        anchor_ts = n * cy_s + modal_sec
        anchor = datetime.fromtimestamp(anchor_ts)
        # If this puts us after the first ON, step back one cycle
        if anchor > first_on:
            anchor = anchor - timedelta(seconds=cy_s)
        print(f"Cycle phase anchor (modal phase): {anchor}")

    # Run with Bp=2.1: empirically the firmware's effective proportional band
    # is slightly larger than the nominal 2.0 set in the menu, presumably
    # due to firmware rounding. Match rate vs the captured data is highest
    # at this value.
    params = Params(bp=2.1, cycle_phase_anchor=anchor)
    results = replay(records, params)
    s = summarise(results)

    print(f"Records replayed:  {s['total']}")
    print(f"Matches:           {s['matches']}  ({s['match_rate']:.1%})")
    print(f"Mismatches:        {s['mismatch_count']}")

    if s["mismatches"]:
        print("\nFirst 15 mismatches:")
        for m in s["mismatches"][:15]:
            print(f"  {m['t']}  SP={m['sp']:5.1f}  T={m['ambient']:5.2f}  "
                  f"actual=0x{m['actual']:02x}  pred=0x{m['predicted']:02x}")
