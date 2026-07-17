/**
 * propwash-core — runs the pilot's Betaflight 4.5.2 in-process against the
 * CineLog35 physics model.
 *
 * Modes:
 *   --server [default]  lockstep UDP protocol server (protocol/): a client
 *                       (Godot, gym, ...) drives sim time with PW_STATE_IN.
 *   --realtime          headless free-run at 60 Hz wall clock (Configurator
 *                       benching; no client needed).
 *
 * MSP/CLI is always available on TCP 5761 (UART1).
 */

#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <thread>

#include "sim/sim.h"
#include "sim/profile_cinelog35.h"
#include "net/server.h"
#include "input/joystick.h"

using namespace SimITL;

int main(int argc, char** argv)
{
    // Line-buffer stdout so boot/diagnostic logs are captured promptly even
    // when redirected to a pipe/file (CI, test harnesses) — default full
    // buffering otherwise loses everything on a SIGTERM without flush.
    setvbuf(stdout, nullptr, _IOLBF, 0);

    const char* eeprom = "eeprom.bin";
    const char* jsDev = nullptr;   // auto-detect
    bool useJs = true;
    bool realtime = false;
    bool calibrate = false;
    double duration = 0.0;
    uint16_t port = 9100;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--eeprom") && i + 1 < argc) {
            eeprom = argv[++i];
        } else if (!strcmp(argv[i], "--port") && i + 1 < argc) {
            port = (uint16_t)atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--js") && i + 1 < argc) {
            jsDev = argv[++i];
        } else if (!strcmp(argv[i], "--no-js")) {
            useJs = false;
        } else if (!strcmp(argv[i], "--js-calibrate")) {
            calibrate = true;
        } else if (!strcmp(argv[i], "--realtime")) {
            realtime = true;
        } else if (!strcmp(argv[i], "--duration") && i + 1 < argc) {
            duration = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--server")) {
            realtime = false;
        } else {
            fprintf(stderr,
                "usage: %s [--server|--realtime|--js-calibrate] [--port N] "
                "[--eeprom path] [--js /dev/input/jsN | --no-js] [--duration s]\n",
                argv[0]);
            return 2;
        }
    }

    // Joystick calibration: standalone, writes the cal file and exits.
    if (calibrate) {
        pw::Joystick jc;
        if (!jc.open(jsDev)) {
            fprintf(stderr, "[pw] no joystick found to calibrate\n");
            return 1;
        }
        printf("[pw] calibrating '%s'\n", jc.name());
        return jc.calibrate(pw::Joystick::defaultCalPath()) ? 0 : 1;
    }

    StateInit init {};
    profileCineLog35(init, eeprom);

    Sim& sim = Sim::getInstance();

    pw::Joystick js;
    if (useJs) {
        if (js.open(jsDev)) {
            js.loadCalibration(pw::Joystick::defaultCalPath());
            printf("[pw] RC source: joystick '%s'\n", js.name());
        } else {
            printf("[pw] no joystick found — RC comes from the client\n");
        }
    }

    if (!realtime) {
        pw::Server server;
        if (!server.start(port)) return 1;
        printf("[pw] lockstep server ready (eeprom=%s)\n", eeprom);
        server.run(sim, init, &js);
        return 0;
    }

    // ---- realtime headless mode
    StateInput input {};
    inputDefaults(input);

    printf("[pw] realtime mode (eeprom=%s)\n", eeprom);
    sim.init(init);
    BF::configureDefaultModes();
    printf("[pw] betaflight initialized; MSP/CLI on tcp:5761 (UART1)\n");

    const double frameDt = 1.0 / 60.0;
    input.delta = (float)frameDt;

    double simTime = 0.0;
    while (duration <= 0.0 || simTime < duration) {
        auto frameStart = std::chrono::steady_clock::now();

        if (js.isOpen()) {
            js.poll();
            memcpy(input.rcData, js.channels(), sizeof(input.rcData));
        }
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
