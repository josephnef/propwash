/**
 * pw-tester — headless integration test for propwash-core.
 *
 * Scenario (all in deterministic lockstep, no wall clock):
 *   phase 0  (0-5 s)   at rest, sticks safe -> gyro calibration completes,
 *                      arming-disable flags must clear
 *   phase 1  (5-6 s)   ARM (ch5 high) -> firmware must report ARMED
 *   phase 2  (6-7 s)   ANGLE (ch6 high)
 *   phase 3  (7-22 s)  throttle managed by a crude altitude controller;
 *                      assert |tilt| < 2 deg and altitude within band
 *
 * Prints STATE_HASH at the end: run the binary twice and compare hashes to
 * verify determinism (each run is a fresh process, fresh eeprom).
 *
 * Exit code 0 = pass.
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "sim/sim.h"
#include "sim/profile_cinelog35.h"
#include "sim/util/vector_math.h"

using namespace SimITL;

static uint64_t fnv1a(uint64_t h, const void* data, size_t len)
{
    const uint8_t* p = (const uint8_t*)data;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}

int main(int argc, char** argv)
{
    const char* eeprom = "pw-tester-eeprom.bin";
    if (argc > 1) eeprom = argv[1];
    remove(eeprom); // always a fresh firmware config -> deterministic

    StateInit init {};
    profileCineLog35(init, eeprom);

    StateInput in {};
    inputDefaults(in);
    in.gyroBaseNoiseAmp = 0.0002f; // keep noise, but deterministic via SimplexNoise

    Sim& sim = Sim::getInstance();
    sim.init(init);
    BF::configureDefaultModes();
    BF::disableRunawayTakeoff();

    const float dt = 1.0f / 250.0f; // 250 Hz client frame, 32 firmware ticks each
    in.delta = dt;

    vec3 position {0, 0, 0};
    uint64_t hash = 1469598103934665603ULL;

    const float T_CAL   = 5.0f;
    const float T_ARM   = 6.0f;
    const float T_ANGLE = 7.0f;
    const float T_END   = 22.0f;
    const float TARGET_ALT = 2.0f;

    bool armedSeen = false;
    bool calCleared = false;
    float maxTiltDeg = 0.0f;
    float minAlt = 1e9f, maxAlt = -1e9f;
    float throttle = -1.0f;

    int frames = (int)(T_END / dt);
    for (int f = 0; f <= frames; f++) {
        float t = f * dt;

        // ---- switch plan: ANGLE on from the start, ARM after calibration
        in.rcData[4] = (t >= T_CAL + 0.2f) ? 1.0f : -1.0f;  // ARM ch5
        in.rcData[5] = 1.0f;                                 // ANGLE ch6

        // ---- crude hover controller (proves BF+physics closed loop)
        const SimState& st = sim.getSimState();
        if (st.armed) {
            float alt = position[1];
            float vy  = st.stateOutput.linearVelocity.y;
            float u = -0.3f + 0.5f * (TARGET_ALT - alt) - 0.4f * vy;
            if (u < -1.0f) u = -1.0f;
            if (u > 0.6f) u = 0.6f;
            // slew-limit: realistic throttle movement, not an instant punch
            float maxStep = 2.0f * dt; // full range in 1 s
            if (u > throttle + maxStep) u = throttle + maxStep;
            if (u < throttle - maxStep) u = throttle - maxStep;
            throttle = u;
        } else {
            throttle = -1.0f;
        }
        in.rcData[2] = throttle;

        sim.update(in);

        const StateOutput& out = sim.getStateUpdate();
        const SimState& state = sim.getSimState();

        // ---- feed outputs back (client-owned pose integration)
        quat q { out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w };
        mat3 basis = quat_to_mat3(q);
        copy(in.rotation, basis);
        in.angularVelocity = out.angularVelocity;
        in.linearVelocity  = out.linearVelocity;

        vec3 vel;
        copy(vel, out.linearVelocity);
        position = position + vel * dt;

        // ---- ground plane: resting on ground kills angular motion too
        // (otherwise motor-noise torque slowly tips the quad over at idle)
        if (position[1] <= 0.0f) {
            position[1] = 0.0f;
            in.contact = 1;
            if (in.linearVelocity.y < 0.0f) in.linearVelocity.y = 0.0f;
            in.angularVelocity = {0.0f, 0.0f, 0.0f};
        } else {
            in.contact = 0;
        }

        // ---- checks
        if (!calCleared && t >= T_CAL && state.armingDisabledFlags == 0) {
            calCleared = true;
            printf("[test] t=%.2f arming-disable flags cleared\n", t);
        }
        if (!armedSeen && state.armed) {
            armedSeen = true;
            printf("[test] t=%.2f ARMED\n", t);
        }
        if (t >= T_ANGLE + 5.0f && t <= T_END) { // settled window
            // tilt = angle between body-up and world-up
            vec3 up = xform(basis, vec3{0, 1, 0});
            float tilt = std::acos(std::fmin(1.0f, std::fmax(-1.0f, up[1]))) * 180.0f / (float)M_PI;
            if (tilt > maxTiltDeg) maxTiltDeg = tilt;
            float alt = position[1];
            if (alt < minAlt) minAlt = alt;
            if (alt > maxAlt) maxAlt = alt;
        }

        // ---- determinism hash over the full output state
        hash = fnv1a(hash, &out, sizeof(out));

        if (f % (int)(1.0f / dt) == 0) {
            printf("[test] t=%5.1f alt=%6.2f thr=%5.2f armed=%d dis=0x%05x motors=%.2f,%.2f,%.2f,%.2f\n",
                   t, position[1], throttle, state.armed ? 1 : 0,
                   (unsigned)state.armingDisabledFlags,
                   state.motorsState[0].pwm, state.motorsState[1].pwm,
                   state.motorsState[2].pwm, state.motorsState[3].pwm);
        }
    }

    printf("STATE_HASH=%016llx\n", (unsigned long long)hash);

    // ---- verdict
    int rc = 0;
    if (!calCleared) { printf("FAIL: arming-disable flags never cleared\n"); rc = 1; }
    if (!armedSeen)  { printf("FAIL: never armed\n"); rc = 1; }
    if (armedSeen) {
        if (maxTiltDeg >= 2.0f) {
            printf("FAIL: max tilt %.2f deg >= 2 deg\n", maxTiltDeg); rc = 1;
        } else {
            printf("[test] max tilt %.3f deg\n", maxTiltDeg);
        }
        if (minAlt < TARGET_ALT - 1.0f || maxAlt > TARGET_ALT + 1.0f) {
            printf("FAIL: altitude out of band [%.2f, %.2f]\n", minAlt, maxAlt); rc = 1;
        } else {
            printf("[test] altitude band [%.2f, %.2f] around target %.1f\n", minAlt, maxAlt, TARGET_ALT);
        }
    }
    printf(rc == 0 ? "PASS\n" : "FAIL\n");
    return rc;
}
