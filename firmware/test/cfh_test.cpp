// Host-side unit test for the call-for-heat P-loop (../src/cfh.h).
//
// cfh.h has no Arduino/hardware dependencies and cfhUpdate() takes `now` as an
// argument, so the whole control loop runs here against simulated time -- no
// ESP32, no radio, no waiting out real 900 s cycles. Build & run on a host:
//
//   g++ -std=c++17 -Wall -Wextra firmware/test/cfh_test.cpp -o /tmp/cfh_test
//   /tmp/cfh_test          # exits 0 on success, 1 on any failure
//
// The expected values below are derived from the documented contract (P-only
// duty, anti-short-cycle clamps, demand-onset pulse, SP-drop cut), NOT read back
// out of the implementation -- so a behavioural regression in cfh.h fails here.
// Constants under test: Bp=2.1, Cy=900s, On_min=Of_min=120s.

#include "../src/cfh.h"

#include <cstdio>
#include <cmath>

static int g_fail = 0;
static int g_checks = 0;

#define CHECK(cond, ...) do {                                   \
    g_checks++;                                                 \
    if (!(cond)) { g_fail++;                                    \
        std::printf("FAIL %s:%d: ", __FILE__, __LINE__);       \
        std::printf(__VA_ARGS__); std::printf("\n"); }         \
} while (0)

static uint32_t SEC(uint32_t s) { return s * 1000UL; }

static bool feq(float a, float b) { return std::fabs(a - b) < 1e-4f; }

// ---------------------------------------------------------------------------

static void test_duty() {
    CHECK(feq(cfhDuty(22.0f, 20.0f), 2.0f / 2.1f), "duty(22,20)");
    CHECK(cfhDuty(20.0f, 22.0f) == 0.0f, "duty negative error clips to 0");
    CHECK(cfhDuty(25.0f, 20.0f) == 1.0f, "duty above band clips to 1");
    CHECK(cfhDuty(20.0f, 20.0f) == 0.0f, "zero error => zero duty");
    CHECK(cfhDuty(0.0f,  20.0f) == 0.0f, "off zone (sp 0) => zero duty");
    CHECK(cfhDuty(NAN,   20.0f) == 0.0f, "NaN setpoint => zero duty");
    CHECK(cfhDuty(22.0f, NAN)   == 0.0f, "NaN ambient => zero duty");
}

static void test_on_duration() {
    CHECK(cfhOnDuration(0.0f) == 0,            "duty 0 => no ON");
    CHECK(cfhOnDuration(1.0f) == SEC(780),     "duty 1 clamps to Cy - Of_min");
    CHECK(cfhOnDuration(0.5f) == SEC(450),     "mid duty passes through");
    CHECK(cfhOnDuration(0.05f) == SEC(120),    "tiny duty floored to On_min");
    CHECK(cfhOnDuration(0.0001f) == SEC(120),  "near-zero duty floored to On_min");
}

// Steady call: sp 22 / amb 20 => duty ~0.952 => ON 780 s of every 900 s cycle.
static void test_steady_call() {
    CfhState c{};
    CHECK(cfhUpdate(&c, 22, 20, 0)          == CFH_CALL, "cycle start calls");
    CHECK(cfhUpdate(&c, 22, 20, SEC(700))   == CFH_CALL, "mid ON-window calls");
    CHECK(cfhUpdate(&c, 22, 20, SEC(780)-1) == CFH_CALL, "just before ON end calls");
    CHECK(cfhUpdate(&c, 22, 20, SEC(780))   == CFH_IDLE, "ON window over => idle");
    CHECK(cfhUpdate(&c, 22, 20, SEC(880))   == CFH_IDLE, "Of_min floor stays idle");
    CHECK(cfhUpdate(&c, 22, 20, SEC(900))   == CFH_CALL, "next cycle re-calls");
    CHECK(cfhUpdate(&c, 22, 20, SEC(1600)) == CFH_CALL, "still calling deep in cycle 2");
}

// Steady idle: ambient above setpoint => never calls, across a cycle boundary.
static void test_steady_idle() {
    CfhState c{};
    CHECK(cfhUpdate(&c, 20, 22, 0)        == CFH_IDLE, "no demand => idle");
    CHECK(cfhUpdate(&c, 20, 22, SEC(500)) == CFH_IDLE, "still idle mid cycle");
    CHECK(cfhUpdate(&c, 20, 22, SEC(900)) == CFH_IDLE, "still idle after boundary");
}

// Demand onset mid-cycle fires an immediate >= On_min pulse, out of phase with Cy.
static void test_demand_onset() {
    CfhState c{};
    CHECK(cfhUpdate(&c, 20, 22, 0)        == CFH_IDLE, "starts idle");
    CHECK(cfhUpdate(&c, 20, 22, SEC(300)) == CFH_IDLE, "still idle just before onset");
    // SP jumps above ambient at t=300s: duty 0 -> positive -> fire On_min pulse.
    CHECK(cfhUpdate(&c, 23, 22, SEC(300)) == CFH_CALL, "onset fires a pulse");
    CHECK(cfhUpdate(&c, 23, 22, SEC(419)) == CFH_CALL, "pulse holds for On_min");
    CHECK(cfhUpdate(&c, 23, 22, SEC(420)) == CFH_IDLE, "pulse expires after On_min");
    // Steady duty is only adopted at the next Cy boundary (t=900s).
    CHECK(cfhUpdate(&c, 23, 22, SEC(900)) == CFH_CALL, "boundary adopts steady duty");
}

// A setpoint drop that makes the error non-positive cuts the active ON pulse
// immediately -- user input bypasses the On_min floor.
static void test_setpoint_drop_cuts_pulse() {
    CfhState c{};
    CHECK(cfhUpdate(&c, 22, 20, 0)          == CFH_CALL, "calling at cycle start");
    CHECK(cfhUpdate(&c, 22, 20, SEC(100))   == CFH_CALL, "still calling at 100 s");
    // User drops SP below ambient at 100 s, far inside the 780 s ON window.
    CHECK(cfhUpdate(&c, 19, 20, SEC(100))   == CFH_IDLE, "SP drop cuts pulse now");
    CHECK(cfhUpdate(&c, 19, 20, SEC(150))   == CFH_IDLE, "stays idle after cut");
}

// An "off" zone is just sp 0 on the wire: it must never call for heat.
static void test_off_zone() {
    CfhState c{};
    CHECK(cfhUpdate(&c, 0, 20, 0)        == CFH_IDLE, "off zone idle at start");
    CHECK(cfhUpdate(&c, 0, 20, SEC(450)) == CFH_IDLE, "off zone idle mid cycle");
    CHECK(cfhUpdate(&c, 0, 20, SEC(900)) == CFH_IDLE, "off zone idle next cycle");
}

// Even a sliver of demand yields a full On_min pulse (anti-short-cycle floor).
static void test_min_on_floor() {
    CfhState c{};
    CHECK(cfhUpdate(&c, 20.1f, 20, 0)        == CFH_CALL, "tiny demand still calls");
    CHECK(cfhUpdate(&c, 20.1f, 20, SEC(119)) == CFH_CALL, "held to On_min");
    CHECK(cfhUpdate(&c, 20.1f, 20, SEC(120)) == CFH_IDLE, "idle once On_min met");
}

int main() {
    test_duty();
    test_on_duration();
    test_steady_call();
    test_steady_idle();
    test_demand_onset();
    test_setpoint_drop_cuts_pulse();
    test_off_zone();
    test_min_on_floor();

    std::printf("\n%d checks, %d failure(s)\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
