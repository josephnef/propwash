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
#include "propwash_protocol.h"

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

// ---------------------------------------------------------------- contacts
// The shared collision hull (protocol header constants): belly + 4 ducts.
struct HullSphere { vec3 center; float r; };
static const HullSphere HULL[5] = {
    {{PW_HULL_BELLY_X, PW_HULL_BELLY_Y, PW_HULL_BELLY_Z}, PW_HULL_BELLY_R},
    {{ PW_HULL_DUCT_XZ, PW_HULL_DUCT_Y,  PW_HULL_DUCT_XZ}, PW_HULL_DUCT_R},
    {{-PW_HULL_DUCT_XZ, PW_HULL_DUCT_Y,  PW_HULL_DUCT_XZ}, PW_HULL_DUCT_R},
    {{ PW_HULL_DUCT_XZ, PW_HULL_DUCT_Y, -PW_HULL_DUCT_XZ}, PW_HULL_DUCT_R},
    {{-PW_HULL_DUCT_XZ, PW_HULL_DUCT_Y, -PW_HULL_DUCT_XZ}, PW_HULL_DUCT_R},
};

/* Build the ground-plane (y=0) contact manifold the way a real client does:
 * test each hull sphere, depenetrate the pose along +y with a 4 mm slop
 * (the residual keeps the core's contact spring loaded so the quad rests on
 * physics, not on a clamp), and report post-depenetration depths.
 *
 * Spheres within MARGIN above the surface are reported too, at depth 0
 * (speculative contacts): the core arms them as one-sided dampers, so a
 * rocking corner is damped BEFORE it re-impacts instead of hitting at full
 * speed during the one-frame detection gap. */
static void groundManifold(vec3& position, const mat3& rotation, StateInput& in)
{
    constexpr float SLOP = 0.004f;
    constexpr float MARGIN = 0.005f;
    struct { vec3 lowBody; float depth; } hits[5];
    int n = 0;
    float maxDepth = 0.0f;
    for (const auto& s : HULL) {
        const vec3 cw = position + xform(rotation, s.center);
        const float depth = s.r - cw[1];
        if (depth > -MARGIN) {
            hits[n].lowBody = s.center + xform_inv(rotation, vec3{0.0f, -s.r, 0.0f});
            hits[n].depth = depth;
            n++;
            if (depth > maxDepth) maxDepth = depth;
        }
    }
    const float dy = maxDepth > SLOP ? maxDepth - SLOP : 0.0f;
    position[1] += dy;

    int m = 0;
    for (int i = 0; i < n && m < 6; i++) {
        const float depth = hits[i].depth - dy;
        auto& c = in.contacts[m++];
        copy(c.pointBody, hits[i].lowBody);
        c.normalWorld = {0.0f, 1.0f, 0.0f};
        c.depth = depth > 0.0f ? depth : 0.0f;
        c.surface = PW_SURF_GROUND;
    }
    in.contactCount = (uint8_t)m;
    in.contact = m > 0 ? 1 : 0;
}

// --------------------------------------------------------------- drop test
/* --drop-test: the "crash outcomes can exist" gate. Drop the disarmed quad
 * 0.5 m onto the contact solver, once level and once inverted. Both must
 * settle dead still on physics alone (no clamp, no velocity zeroing) — and
 * the inverted one must STAY inverted instead of magically righting itself,
 * because resting upside-down after a crash is a real outcome. */
static int dropScenario(Sim& sim, const StateInit& init, bool inverted)
{
    StateInput in {};
    inputDefaults(in);
    in.gyroBaseNoiseAmp = 0.0002f;
    const float dt = 1.0f / 250.0f;
    in.delta = dt;

    mat3 basis = identity;
    if (inverted) {
        // 180 deg roll about z: body-up points at world -y
        basis = mat3{vec3{-1, 0, 0}, vec3{0, -1, 0}, vec3{0, 0, 1}};
    }
    vec3 position {0.0f, 0.5f, 0.0f};
    copy(in.rotation, basis);

    const float T_END = 3.0f;
    const int frames = (int)(T_END / dt);
    float maxRestV = 0.0f, maxRestW = 0.0f;
    bool touched = false;

    for (int f = 0; f <= frames; f++) {
        const float t = f * dt;
        sim.update(in);

        const StateOutput& out = sim.getStateUpdate();
        quat q { out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w };
        basis = quat_to_mat3(q);
        copy(in.rotation, basis);
        in.angularVelocity = out.angularVelocity; // echo; core-authoritative
        in.linearVelocity  = out.linearVelocity;

        vec3 vel;
        copy(vel, out.linearVelocity);
        position = position + vel * dt;

        groundManifold(position, basis, in);
        if (in.contactCount > 0) touched = true;

        if (t >= T_END - 1.0f) { // settled window: dead still, no jitter
            vec3 v, w;
            copy(v, out.linearVelocity);
            copy(w, out.angularVelocity);
            if (length(v) > maxRestV) maxRestV = length(v);
            if (length(w) > maxRestW) maxRestW = length(w);
        }
    }

    const vec3 up = get_axis(basis, 1); // body-up in world frame
    printf("[drop] %s: rest |v|max=%.5f m/s |w|max=%.5f rad/s bodyUp.y=%+.3f alt=%.4f\n",
           inverted ? "inverted" : "level   ", maxRestV, maxRestW, up[1], position[1]);

    int rc = 0;
    if (!touched) { printf("FAIL: never touched the ground\n"); rc = 1; }
    if (maxRestV > 1e-3f || maxRestW > 1e-3f) {
        printf("FAIL: did not settle (|v|max=%.5f, |w|max=%.5f)\n", maxRestV, maxRestW);
        rc = 1;
    }
    if (!inverted && up[1] < 0.99f) {
        printf("FAIL: level drop should rest level (bodyUp.y=%.3f)\n", up[1]);
        rc = 1;
    }
    /* Inverted rest is belly sphere + two ducts — a real tail-up tilt, so
     * only require "clearly upside-down", not perfectly flat. */
    if (inverted && up[1] > -0.7f) {
        printf("FAIL: inverted drop righted itself (bodyUp.y=%.3f)\n", up[1]);
        rc = 1;
    }
    return rc;
}

static int dropTest()
{
    const char* eeprom = "pw-tester-drop-eeprom.bin";
    remove(eeprom);

    StateInit init {};
    profileCineLog35(init, eeprom);

    Sim& sim = Sim::getInstance();
    sim.init(init);
    BF::configureDefaultModes();
    BF::disableRunawayTakeoff();

    int rc = dropScenario(sim, init, false);
    sim.reset(init); // fresh firmware + physics for the second scenario
    rc |= dropScenario(sim, init, true);

    printf(rc == 0 ? "PASS\n" : "FAIL\n");
    return rc;
}

// -------------------------------------------------------------- crash test
/* --crash-test: the full crash lifecycle. Rest never damages; a 10 m drop is
 * a structural crash (all motors >= 0.85 damage, CRASHED latch); a crashed
 * quad physically cannot hover (1.12 N max thrust < 3.24 N weight — the
 * failure is the physics, not a scripted "you lose"); REPAIR clears it and
 * the same controller then hovers normally. Deterministic lockstep, no
 * wall clock. */
static int crashTest()
{
    const char* eeprom = "pw-tester-crash-eeprom.bin";
    remove(eeprom);

    StateInit init {};
    profileCineLog35(init, eeprom);

    StateInput in {};
    inputDefaults(in);
    in.gyroBaseNoiseAmp = 0.0002f;
    const float dt = 1.0f / 250.0f;
    in.delta = dt;

    Sim& sim = Sim::getInstance();
    sim.init(init);
    BF::configureDefaultModes();
    BF::disableRunawayTakeoff();

    mat3 basis = identity;
    vec3 position {0.0f, PW_HULL_REST_H, 0.0f}; // starts AT REST on the pad
    copy(in.rotation, basis);

    int rc = 0;
    float throttle = -1.0f;
    bool crashSeen = false;
    float crashSeenAt = -1.0f;
    float firstContactAt = -1.0f;
    float maxAltCrashed = 0.0f;
    float minAltHover = 1e9f, maxAltHover = -1e9f;
    bool armedSeen = false, rearmedSeen = false;

    /* timeline:
     *  0-6    settle + gyro cal at rest      -> zero damage the whole time
     *  6      teleport to 10 m, free-fall    -> structural crash on impact
     *  8-9    arm (ch5 up)
     *  9-19   hover attempt on wrecked quad  -> altitude must stay < 1 m
     *  19-20  disarm, REPAIR
     *  20-21  re-arm
     *  21-36  hover                          -> [1, 3] m band once settled */
    const float T_DROP = 6.0f, T_ARM1 = 8.0f, T_DISARM = 19.0f;
    const float T_REPAIR = 19.5f, T_ARM2 = 20.0f, T_END = 36.0f;

    const int frames = (int)(T_END / dt);
    for (int f = 0; f <= frames; f++) {
        const float t = f * dt;

        if (f == (int)(T_DROP / dt)) {
            position = {0.0f, 10.0f, 0.0f};
        }
        if (f == (int)(T_REPAIR / dt)) {
            sim.command(CommandType::Repair);
        }

        const bool armWindow = (t >= T_ARM1 && t < T_DISARM) || t >= T_ARM2;
        in.rcData[4] = armWindow ? 1.0f : -1.0f; // ARM ch5
        in.rcData[5] = 1.0f;                     // ANGLE ch6

        const SimState& st = sim.getSimState();
        if (st.armed) {
            float u = -0.3f + 0.5f * (2.0f - position[1]) - 0.4f * st.stateOutput.linearVelocity.y;
            u = clamp(u, -1.0f, 0.6f);
            const float maxStep = 2.0f * dt;
            throttle = clamp(u, throttle - maxStep, throttle + maxStep);
        } else {
            throttle = -1.0f;
        }
        in.rcData[2] = throttle;

        sim.update(in);

        const StateOutput& out = sim.getStateUpdate();
        quat q { out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w };
        basis = quat_to_mat3(q);
        copy(in.rotation, basis);
        in.angularVelocity = out.angularVelocity;
        in.linearVelocity  = out.linearVelocity;

        vec3 vel;
        copy(vel, out.linearVelocity);
        position = position + vel * dt;
        groundManifold(position, basis, in);

        float maxDmg = 0.0f, minDmg = 1.0f;
        for (int i = 0; i < 4; i++) {
            maxDmg = std::max(maxDmg, out.propDamage[i]);
            minDmg = std::min(minDmg, out.propDamage[i]);
        }

        // phase asserts
        if (t < T_DROP) {
            if (maxDmg > 0.0f || out.crashFlags != 0) {
                printf("FAIL: damage at rest (t=%.2f dmg=%.3f flags=%d)\n",
                       t, maxDmg, out.crashFlags);
                rc = 1;
                break;
            }
        }
        if (t > T_DROP && firstContactAt < 0.0f && in.contactCount > 0) {
            firstContactAt = t;
        }
        if (!crashSeen && (out.crashFlags & 1)) {
            crashSeen = true;
            crashSeenAt = t;
            if (minDmg < 0.85f) {
                printf("FAIL: crash latch with min damage %.2f < 0.85\n", minDmg);
                rc = 1;
            }
            for (int i = 0; i < 4; i++) {
                if (!(out.motorStatus[i] & MotorStatus::MotorDamaged)) {
                    printf("FAIL: motor %d not flagged MotorDamaged after crash\n", i);
                    rc = 1;
                }
            }
        }
        if (st.armed && t < T_DISARM) armedSeen = true;
        if (st.armed && t >= T_ARM2) rearmedSeen = true;
        if (t >= T_ARM1 && t < T_DISARM) {
            maxAltCrashed = std::max(maxAltCrashed, position[1]);
        }
        if (t >= T_ARM2 + 8.0f) { // settled window of the post-repair hover
            minAltHover = std::min(minAltHover, position[1]);
            maxAltHover = std::max(maxAltHover, position[1]);
        }
        if (f == (int)(T_ARM2 / dt)) { // just after repair: all clear
            if (maxDmg != 0.0f || out.crashFlags != 0) {
                printf("FAIL: repair did not clear (dmg=%.3f flags=%d)\n",
                       maxDmg, out.crashFlags);
                rc = 1;
            }
        }

        if (f % (int)(2.0f / dt) == 0) {
            printf("[crash] t=%5.1f alt=%6.2f thr=%5.2f armed=%d dmg=%.2f flags=%d\n",
                   t, position[1], throttle, st.armed ? 1 : 0, maxDmg,
                   (int)out.crashFlags);
        }
    }

    if (!crashSeen) { printf("FAIL: 10 m drop never latched CRASHED\n"); rc = 1; }
    if (crashSeen && firstContactAt > 0.0f && crashSeenAt - firstContactAt > 0.5f) {
        printf("FAIL: crash latched %.2f s after contact\n", crashSeenAt - firstContactAt);
        rc = 1;
    }
    if (!armedSeen) { printf("FAIL: never armed on the wrecked quad\n"); rc = 1; }
    if (maxAltCrashed >= 1.0f) {
        printf("FAIL: crashed quad climbed to %.2f m (>= 1.0)\n", maxAltCrashed);
        rc = 1;
    } else {
        printf("[crash] wrecked-quad hover attempt peaked at %.2f m — grounded, as physics demands\n",
               maxAltCrashed);
    }
    if (!rearmedSeen) { printf("FAIL: never re-armed after repair\n"); rc = 1; }
    if (minAltHover < 1.0f || maxAltHover > 3.0f) {
        printf("FAIL: post-repair hover out of band [%.2f, %.2f]\n", minAltHover, maxAltHover);
        rc = 1;
    } else {
        printf("[crash] post-repair hover band [%.2f, %.2f]\n", minAltHover, maxAltHover);
    }
    printf(rc == 0 ? "PASS\n" : "FAIL\n");
    return rc;
}

// --------------------------------------------------------------- dshot test
/* --dshot-test: the virtual DSHOT ESC end to end. With PROPWASH_DSHOT=1 the
 * forced-PWM override is skipped, so a fresh eeprom boots the Betaflight
 * default protocol (DSHOT600 once USE_DSHOT is compiled). Asserts:
 *   1. boot grace clears and the quad arms + hovers — proves the
 *      motor-enable re-stamp, dshotStreamingCommandsAreEnabled(), and the
 *      virtual ESC's throttle mapping (48..2047 -> thrust);
 *   2. SPIN_DIRECTION commands sent through the STOCK dshot_command.c queue
 *      (repeats, delays, all-motors-idle gating) land in pw_motor_dir. */
static int dshotTest()
{
const char* eeprom = "pw-tester-dshot-eeprom.bin";
    remove(eeprom);

    StateInit init {};
    profileCineLog35(init, eeprom);

    StateInput in {};
    inputDefaults(in);
    in.gyroBaseNoiseAmp = 0.0002f;
    const float dt = 1.0f / 250.0f;
    in.delta = dt;

    Sim& sim = Sim::getInstance();
    sim.init(init);
    BF::configureDefaultModes();
    BF::disableRunawayTakeoff();

    mat3 basis = identity;
    vec3 position {0.0f, PW_HULL_REST_H, 0.0f};
    copy(in.rotation, basis);

    int rc = 0;
    float throttle = -1.0f;
    bool armedSeen = false;
    float minAlt = 1e9f, maxAlt = -1e9f;

    /* timeline: cal+grace to 6 s, arm, hover to 20 s, disarm, then the
     * spin-direction command exchange while resting idle */
    const float T_ARM = 6.0f, T_DISARM = 20.0f, T_REV = 21.0f, T_NORM = 22.0f;
    const float T_END = 23.0f;

    const int frames = (int)(T_END / dt);
    for (int f = 0; f <= frames; f++) {
        const float t = f * dt;

        in.rcData[4] = (t >= T_ARM && t < T_DISARM) ? 1.0f : -1.0f;
        in.rcData[5] = 1.0f;

        const SimState& st = sim.getSimState();
        if (st.armed) {
            float u = -0.3f + 0.5f * (2.0f - position[1]) - 0.4f * st.stateOutput.linearVelocity.y;
            u = clamp(u, -1.0f, 0.6f);
            const float maxStep = 2.0f * dt;
            throttle = clamp(u, throttle - maxStep, throttle + maxStep);
        } else {
            throttle = -1.0f;
        }
        in.rcData[2] = throttle;

        if (f == (int)(T_REV / dt)) {
            BF::sendSpinDirectionCommand(true);
        }
        if (f == (int)(T_NORM / dt)) {
            for (int i = 0; i < 4 && rc == 0; i++) {
                if (BF::motorSpinDirection(i) != -1) {
                    printf("FAIL: motor %d dir %d after REVERSED\n",
                           i, BF::motorSpinDirection(i));
                    rc = 1;
                }
            }
            BF::sendSpinDirectionCommand(false);
        }

        sim.update(in);

        const StateOutput& out = sim.getStateUpdate();
        quat q { out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w };
        basis = quat_to_mat3(q);
        copy(in.rotation, basis);
        in.angularVelocity = out.angularVelocity;
        in.linearVelocity  = out.linearVelocity;

        vec3 vel;
        copy(vel, out.linearVelocity);
        position = position + vel * dt;
        groundManifold(position, basis, in);

        if (st.armed) armedSeen = true;
        if (t >= T_ARM + 8.0f && t < T_DISARM) {
            minAlt = std::min(minAlt, position[1]);
            maxAlt = std::max(maxAlt, position[1]);
        }
        if (f % (int)(2.0f / dt) == 0) {
            printf("[dshot] t=%5.1f alt=%6.2f armed=%d dis=0x%05x dir=%+d%+d%+d%+d\n",
                   t, position[1], st.armed ? 1 : 0,
                   (unsigned)st.armingDisabledFlags,
                   BF::motorSpinDirection(0), BF::motorSpinDirection(1),
                   BF::motorSpinDirection(2), BF::motorSpinDirection(3));
        }
    }

    for (int i = 0; i < 4; i++) {
        if (BF::motorSpinDirection(i) != 1) {
            printf("FAIL: motor %d dir %d after NORMAL\n",
                   i, BF::motorSpinDirection(i));
            rc = 1;
        }
    }
    if (!armedSeen) { printf("FAIL: never armed under DSHOT\n"); rc = 1; }
    if (armedSeen && (minAlt < 1.0f || maxAlt > 3.0f)) {
        printf("FAIL: dshot hover out of band [%.2f, %.2f]\n", minAlt, maxAlt);
        rc = 1;
    } else if (armedSeen) {
        printf("[dshot] hover band [%.2f, %.2f] on the virtual DSHOT ESC\n", minAlt, maxAlt);
    }
    printf(rc == 0 ? "PASS\n" : "FAIL\n");
    return rc;
}

// -------------------------------------------------------------- turtle test
/* --turtle-test: crashflip end to end, as an emergent contact-solver
 * maneuver. The quad settles INVERTED, the turtle box (ch7) goes active,
 * arming reverses the props through the stock DSHOT command queue, and a
 * held roll stick pivots the quad over its duct edge until it is upright —
 * at which point the "pilot" releases the stick, disarms, and re-arms
 * normally (directions restored). small_angle=180 is set like the real
 * dump does — arming inverted is the turtle prerequisite. */
static int turtleTest()
{
const char* eeprom = "pw-tester-turtle-eeprom.bin";
    remove(eeprom);

    StateInit init {};
    profileCineLog35(init, eeprom);

    StateInput in {};
    inputDefaults(in);
    in.gyroBaseNoiseAmp = 0.0002f;
    const float dt = 1.0f / 250.0f;
    in.delta = dt;

    Sim& sim = Sim::getInstance();
    sim.init(init);
    // small_angle=180 like the real dump — allows arming inverted. It is
    // cached as a cosine at imu init, so a RAM poke alone is not enough:
    // persist and reboot, the same way a pilot's saved config applies.
    BF::setSmallAngle(180);
    BF::saveConfig();
    sim.reset(init);

    // start inverted, dropped just above the pad (settles on belly + ducts)
    mat3 basis = mat3{vec3{-1, 0, 0}, vec3{0, -1, 0}, vec3{0, 0, 1}};
    vec3 position {0.0f, 0.3f, 0.0f};
    copy(in.rotation, basis);

    int rc = 0;
    bool turtleSeen = false;
    bool rearmedSeen = false;
    bool righted = false;
    float maxUp = -1.0f;

    /* burst-and-coast stick technique, like a real pilot: a marginal-
     * authority whoop (2 reversed motors ~ 1.2x weight) stalls at the
     * balance point under a HELD stick — the thrust vector turns downward
     * as the quad comes up. Short bursts build pivot momentum instead. */
    const float BURST_ON = 0.5f, BURST_OFF = 0.35f;

    const float T_BOX = 5.0f, T_ARM = 5.5f, T_DISARM = 9.0f, T_REARM = 11.0f;
    const float T_END = 13.0f;

    const int frames = (int)(T_END / dt);
    for (int f = 0; f <= frames; f++) {
        const float t = f * dt;
        const bool flipPhase = t >= T_ARM && t < T_DISARM;

        in.rcData[4] = (flipPhase || t >= T_REARM) ? 1.0f : -1.0f; // ARM
        in.rcData[5] = 1.0f;                                       // ANGLE
        in.rcData[6] = (t >= T_BOX && t < T_DISARM) ? 1.0f : -1.0f; // TURTLE box
        in.rcData[2] = -1.0f;                                      // throttle low
        // burst-and-coast until past horizontal, then hands off
        float roll = 0.0f;
        if (flipPhase && !righted) {
            const float phase = fmodf(t - T_ARM, BURST_ON + BURST_OFF);
            roll = phase < BURST_ON ? 1.0f : 0.0f;
        }
        in.rcData[0] = roll;

        sim.update(in);

        const StateOutput& out = sim.getStateUpdate();
        const SimState& st = sim.getSimState();
        quat q { out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w };
        basis = quat_to_mat3(q);
        copy(in.rotation, basis);
        in.angularVelocity = out.angularVelocity;
        in.linearVelocity  = out.linearVelocity;

        vec3 vel;
        copy(vel, out.linearVelocity);
        position = position + vel * dt;
        groundManifold(position, basis, in);

        const float upY = get_axis(basis, 1)[1];
        if (upY > maxUp) maxUp = upY;
        if (flipPhase && upY > 0.5f) righted = true;
        if (out.crashFlags & 4) turtleSeen = true;
        if (t >= T_REARM && st.armed && !(out.crashFlags & 4)) rearmedSeen = true;

        if (f % (int)(1.0f / dt) == 0) {
            printf("[turtle] t=%5.1f upY=%+.2f armed=%d flags=%d dir=%+d%+d%+d%+d dis=0x%05x\n",
                   t, upY, st.armed ? 1 : 0, (int)out.crashFlags,
                   BF::motorSpinDirection(0), BF::motorSpinDirection(1),
                   BF::motorSpinDirection(2), BF::motorSpinDirection(3),
                   (unsigned)st.armingDisabledFlags);
        }
    }

    if (!turtleSeen) { printf("FAIL: turtle mode never activated (crash_flags bit2)\n"); rc = 1; }
    if (maxUp < 0.9f) {
        printf("FAIL: never righted itself (max bodyUp.y %.2f)\n", maxUp);
        rc = 1;
    } else {
        printf("[turtle] righted: max bodyUp.y %.2f\n", maxUp);
    }
    if (!rearmedSeen) { printf("FAIL: never re-armed normally after the flip\n"); rc = 1; }
    for (int i = 0; i < 4; i++) {
        if (BF::motorSpinDirection(i) != 1) {
            printf("FAIL: motor %d still reversed after normal re-arm\n", i);
            rc = 1;
        }
    }
    printf(rc == 0 ? "PASS\n" : "FAIL\n");
    return rc;
}

// ---------------------------------------------------------- rpm filter test
/* --rpmfilter-test: bidirectional-DSHOT eRPM telemetry end to end. The
 * virtual ESC encodes physics-true rpm into the real eRPM period frames;
 * the STOCK stack decodes them and feeds the RPM filter — the firmware
 * flies with the pilot's actual gyro-filter configuration. Asserts the
 * firmware-side telemetry rpm matches the physics rpm and that the filter
 * is genuinely enabled while a hover stays stable. */
static int rpmFilterTest()
{
const char* eeprom = "pw-tester-rpmfilter-eeprom.bin";
    remove(eeprom);

    StateInit init {};
    profileCineLog35(init, eeprom);

    StateInput in {};
    inputDefaults(in);
    in.gyroBaseNoiseAmp = 0.0002f;
    const float dt = 1.0f / 250.0f;
    in.delta = dt;

    Sim& sim = Sim::getInstance();
    sim.init(init);
    // dshot_bidir like the real dump, persisted, then a clean reboot: the
    // RPM filter samples the flag at gyro init
    BF::setDshotBidir(true);
    BF::saveConfig();
    sim.reset(init);

    mat3 basis = identity;
    vec3 position {0.0f, PW_HULL_REST_H, 0.0f};
    copy(in.rotation, basis);

    int rc = 0;
    float throttle = -1.0f;
    bool armedSeen = false;
    float minAlt = 1e9f, maxAlt = -1e9f;

    const float T_ARM = 6.0f, T_CHECK = 15.0f, T_END = 16.0f;

    const int frames = (int)(T_END / dt);
    for (int f = 0; f <= frames; f++) {
        const float t = f * dt;

        in.rcData[4] = (t >= T_ARM) ? 1.0f : -1.0f;
        in.rcData[5] = 1.0f;

        const SimState& st = sim.getSimState();
        if (st.armed) {
            armedSeen = true;
            float u = -0.3f + 0.5f * (2.0f - position[1]) - 0.4f * st.stateOutput.linearVelocity.y;
            u = clamp(u, -1.0f, 0.6f);
            const float maxStep = 2.0f * dt;
            throttle = clamp(u, throttle - maxStep, throttle + maxStep);
        } else {
            throttle = -1.0f;
        }
        in.rcData[2] = throttle;

        sim.update(in);

        const StateOutput& out = sim.getStateUpdate();
        quat q { out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w };
        basis = quat_to_mat3(q);
        copy(in.rotation, basis);
        in.angularVelocity = out.angularVelocity;
        in.linearVelocity  = out.linearVelocity;

        vec3 vel;
        copy(vel, out.linearVelocity);
        position = position + vel * dt;
        groundManifold(position, basis, in);

        if (t >= T_ARM + 8.0f) {
            minAlt = std::min(minAlt, position[1]);
            maxAlt = std::max(maxAlt, position[1]);
        }

        if (f == (int)(T_CHECK / dt)) {
            for (int i = 0; i < 4; i++) {
                const float physicsRpm = st.motorsState[i].rpm;
                const float fcRpm = BF::dshotTelemetryRpm(i);
                printf("[rpmfilter] motor %d: physics %.0f rpm, firmware sees %.0f rpm\n",
                       i, physicsRpm, fcRpm);
                if (physicsRpm < 1000.0f) {
                    printf("FAIL: motor %d not spinning in hover?\n", i);
                    rc = 1;
                } else if (std::fabs(fcRpm - physicsRpm) / physicsRpm > 0.05f) {
                    printf("FAIL: telemetry rpm off by >5%%\n");
                    rc = 1;
                }
            }
            if (!BF::dshotTelemetryActive()) {
                printf("FAIL: dshot telemetry not active\n");
                rc = 1;
            }
            if (!BF::rpmFilterEnabled()) {
                printf("FAIL: RPM filter not enabled\n");
                rc = 1;
            }
        }
    }

    if (!armedSeen) { printf("FAIL: never armed\n"); rc = 1; }
    if (armedSeen && (minAlt < 1.0f || maxAlt > 3.0f)) {
        printf("FAIL: hover out of band [%.2f, %.2f] with RPM filter active\n", minAlt, maxAlt);
        rc = 1;
    } else if (armedSeen) {
        printf("[rpmfilter] hover band [%.2f, %.2f] with the RPM filter running\n", minAlt, maxAlt);
    }
    printf(rc == 0 ? "PASS\n" : "FAIL\n");
    return rc;
}

int main(int argc, char** argv)
{
    if (argc > 1 && strcmp(argv[1], "--drop-test") == 0) {
        return dropTest();
    }
    if (argc > 1 && strcmp(argv[1], "--turtle-test") == 0) {
        return turtleTest();
    }
    if (argc > 1 && strcmp(argv[1], "--rpmfilter-test") == 0) {
        return rpmFilterTest();
    }
    if (argc > 1 && strcmp(argv[1], "--crash-test") == 0) {
        return crashTest();
    }
    if (argc > 1 && strcmp(argv[1], "--dshot-test") == 0) {
        return dshotTest();
    }
    const char* eeprom = "pw-tester-eeprom.bin";
    if (argc > 1) eeprom = argv[1];
    remove(eeprom); // always a fresh firmware config -> deterministic

    StateInit init {};
    profileCineLog35(init, eeprom);

    StateInput in {};
    inputDefaults(in);
    in.gyroBaseNoiseAmp = 0.0002f; // noise on, but from the sim-owned seeded RNG

    Sim& sim = Sim::getInstance();
    sim.init(init);
    BF::configureDefaultModes();
    BF::disableRunawayTakeoff();

    const float dt = 1.0f / 250.0f; // 250 Hz client frame = 80 firmware ticks each
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

        // ---- ground contact manifold (shared hull, analytic plane); the
        // core resolves rest/impacts as forces — no velocity zeroing here
        groundManifold(position, basis, in);

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
