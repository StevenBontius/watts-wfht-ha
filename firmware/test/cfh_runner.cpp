// Thin stdin/stdout driver around the real call-for-heat loop (../src/cfh.h),
// so the Python equivalence harness (tools/cfh_equiv_test.py) can run the exact
// firmware logic and diff it against wfht_emulator.py sample-for-sample.
//
//   build:  g++ -std=c++17 -O2 firmware/test/cfh_runner.cpp -o cfh_runner
//   stdin:  one "<now_ms> <sp> <amb>" triple per line, in time order
//   stdout: the byte-20 flag (decimal) for each line
//
// A single CfhState persists across lines -- i.e. one zone advanced over time,
// exactly as serviceControlLoops drives it on-device.

#include "../src/cfh.h"
#include <cstdio>

int main() {
    CfhState c{};
    unsigned long now_ms;
    double sp, amb;
    while (std::scanf("%lu %lf %lf", &now_ms, &sp, &amb) == 3) {
        uint8_t flag = cfhUpdate(&c, (float)sp, (float)amb, (uint32_t)now_ms);
        std::printf("%u\n", (unsigned)flag);
    }
    return 0;
}
