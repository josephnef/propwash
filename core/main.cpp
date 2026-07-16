/**
 * propwash-core — M0/M1 realtime headless runner.
 *
 * Boots Betaflight 4.5.2 in-process with the CineLog35 physics profile and
 * ticks it in wall-clock time. Betaflight Configurator / MSP tools can
 * connect to TCP 5761 (UART1). The UDP protocol server arrives with M2.
 */

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include "sim/sim.h"
#include "sim/profile_cinelog35.h"

using namespace SimITL;

int main(int argc, char** argv)
{
    const char* eeprom = "eeprom.bin";
    double duration = 0.0; // seconds; 0 = run forever

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--eeprom") && i + 1 < argc) {
            eeprom = argv[++i];
        } else if (!strcmp(argv[i], "--duration") && i + 1 < argc) {
            duration = atof(argv[++i]);
        } else {
            fprintf(stderr, "usage: %s [--eeprom path] [--duration seconds]\n", argv[0]);
            return 2;
        }
    }

    StateInit init {};
    profileCineLog35(init, eeprom);

    StateInput input {};
    inputDefaults(input);

    printf("[pw] propwash-core starting (eeprom=%s)\n", eeprom);
    Sim& sim = Sim::getInstance();
    sim.init(init);
    printf("[pw] betaflight initialized; MSP/CLI on tcp:5761 (UART1)\n");

    const double frameDt = 1.0 / 60.0;
    input.delta = (float)frameDt;

    double simTime = 0.0;
    while (duration <= 0.0 || simTime < duration) {
        auto frameStart = std::chrono::steady_clock::now();

        sim.update(input);
        simTime += frameDt;

        std::this_thread::sleep_until(
            frameStart + std::chrono::microseconds((int64_t)(frameDt * 1e6)));
    }

    const SimState& st = sim.getSimState();
    printf("[pw] done. sim_time=%.2fs armed=%d arming_disable=0x%x\n",
           simTime, st.armed ? 1 : 0, (unsigned)st.armingDisabledFlags);
    return 0;
}
