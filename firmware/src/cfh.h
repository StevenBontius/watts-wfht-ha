// ---------------------------------------------------------------------------
// Call-for-heat P-loop (frame byte 20), ported from tools/wfht_emulator.py.
// ---------------------------------------------------------------------------
//   duty = clip((SP - T) / Bp, 0, 1)                     # P-only, no integral
//   ON per Cy = clip(duty * Cy, On_min, Cy - Of_min)     # anti-short-cycle
// Duty is sampled once per Cy boundary and held; a demand onset (0 -> positive)
// fires an out-of-cycle ON pulse of >= On_min, and a setpoint drop that makes
// the error negative cuts the active pulse immediately (user input bypasses
// On_min). The flag is computed live per zone (cfhUpdate) and sent as byte 20.
//
// This unit is deliberately free of Arduino/hardware dependencies: cfhUpdate
// takes `now` as an argument instead of calling millis(), so the whole loop is
// host-testable with simulated time (see firmware/test/cfh_test.cpp). main.cpp
// embeds a CfhState in each ZoneBinding and drives it from the scheduler.
//
// NOTE: the WFHC-MASTER receiver self-regulates on temperature and IGNORES this
// flag (confirmed on hardware), so for that receiver the loop has no actuation
// effect -- it is wired live only for a dumb receiver that acts on call-for-heat.
#ifndef WATTS_CFH_H
#define WATTS_CFH_H

#include <stdint.h>
#include <math.h>

static const float    CFH_BP_C      = 2.1f;            // proportional band, deg C (effective firmware value)
static const uint32_t CFH_CY_MS     = 900UL * 1000;    // cycle time
static const uint32_t CFH_ON_MIN_MS = 120UL * 1000;    // min ON  (anti-short-cycle floor)
static const uint32_t CFH_OF_MIN_MS = 120UL * 1000;    // min OFF (ceiling on ON within a cycle)
static const uint8_t  CFH_CALL = 0x64;                 // byte-20 value when calling for heat
static const uint8_t  CFH_IDLE = 0x00;                 // byte-20 value when idle

// Per-zone control-loop state. All runtime, zero-initialised (= idle, un-anchored);
// the loop free-runs its own Cy phase from first data. Embedded in ZoneBinding.
struct CfhState {
    bool     ctrlInit;        // cycle phase established
    uint32_t cycleStartMs;    // start of the current Cy window
    uint32_t onPulseEndMs;    // absolute millis the ON pulse ends
    float    prevDuty;        // previous-sample duty (demand-onset edge)
    uint8_t  flag;            // current byte-20 flag (CFH_IDLE / CFH_CALL)
};

// Target duty from error, P-only (emulator._duty with cp=a0=0, j1="Hot").
static inline float cfhDuty(float sp, float tAmb) {
    if (isnan(sp) || isnan(tAmb)) return 0.0f;
    float duty = (sp - tAmb) / CFH_BP_C;
    if (duty < 0.0f) return 0.0f;
    if (duty > 1.0f) return 1.0f;
    return duty;
}

// Raw duty -> ON duration (ms) with the anti-short-cycle clamps (emulator._on_duration).
static inline uint32_t cfhOnDuration(float duty) {
    if (duty <= 0.0f) return 0;
    uint32_t raw = (uint32_t)(duty * (float)CFH_CY_MS);
    if (raw < CFH_ON_MIN_MS)             return CFH_ON_MIN_MS;
    if (raw > CFH_CY_MS - CFH_OF_MIN_MS) return CFH_CY_MS - CFH_OF_MIN_MS;
    return raw;
}

// Advance one zone's call-for-heat state to `now` and return its byte-20 flag.
// Mirrors emulator.Emulator.update: sample duty at each Cy boundary and hold it,
// fire an out-of-cycle ON pulse on a demand onset, and cut the active pulse the
// moment a setpoint drop makes the error negative. `sp` is the on-wire setpoint,
// so an "off" zone (sp 0) collapses to no demand without a special case. Signed
// millis() deltas tolerate the 49-day wrap (same idiom as the stale failsafe).
static inline uint8_t cfhUpdate(CfhState *c, float sp, float tAmb, uint32_t now) {
    if (!c->ctrlInit) {                       // first data: free-run a fresh cycle
        c->ctrlInit     = true;
        c->cycleStartMs = now;
        c->prevDuty     = 0.0f;
        c->onPulseEndMs = now + cfhOnDuration(cfhDuty(sp, tAmb));
    }
    // Roll forward through any Cy boundaries crossed, resampling duty at each.
    while ((int32_t)(now - c->cycleStartMs) >= (int32_t)CFH_CY_MS) {
        c->cycleStartMs += CFH_CY_MS;
        c->onPulseEndMs  = c->cycleStartMs + cfhOnDuration(cfhDuty(sp, tAmb));
    }
    float duty = cfhDuty(sp, tAmb);
    // Demand onset: duty 0 -> positive while no pulse is active -> fire >= On_min.
    if (duty > 0.0f && c->prevDuty <= 0.0f &&
        (int32_t)(now - c->onPulseEndMs) >= 0) {
        c->onPulseEndMs = now + CFH_ON_MIN_MS;
    }
    c->prevDuty = duty;
    // Flag for `now`: calling while inside the ON pulse, unless the error just
    // went non-positive (user dropped SP) -- then cut the pulse immediately.
    if ((int32_t)(now - c->onPulseEndMs) < 0) {
        if (duty <= 0.0f) { c->onPulseEndMs = now; c->flag = CFH_IDLE; }
        else                c->flag = CFH_CALL;
    } else {
        c->flag = CFH_IDLE;
    }
    return c->flag;
}

#endif // WATTS_CFH_H
