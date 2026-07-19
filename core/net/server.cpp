#include "server.h"
#include <chrono>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#include <mach-o/loader.h>
#include <mach-o/ldsyms.h>
#include <mach/vm_prot.h>
#elif defined(__linux__)
// linker section boundaries — file scope + C linkage, or the references
// mangle to namespace-qualified symbols and fail to link (see
// static_snapshot.cpp for the same trap)
extern "C" {
extern char __data_start[];
extern char _edata[];
extern char __bss_start[];
extern char _end[];
}
#endif

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <basetsd.h>
// dyad also uses Winsock; WSAStartup is refcounted so both can init/cleanup.
// The Winsock headers don't pull in POSIX ssize_t; define it (guarded by
// MinGW's own macro so we don't clash if <sys/types.h> already provided it).
#ifndef _SSIZE_T_DEFINED
#define _SSIZE_T_DEFINED
typedef SSIZE_T ssize_t;
#endif
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "sim/profile_cinelog35.h"
#include "sim/util/vector_math.h"

namespace pw {

using namespace SimITL;

// Wire layout guards: every codec (Python struct strings, GDScript byte
// counts) mirrors these sizes by hand — catch drift at compile time.
static_assert(sizeof(PwContact) == 29, "PwContact wire size changed");
static_assert(sizeof(PwStateIn) == 308, "PwStateIn wire size changed");
static_assert(sizeof(PwStateOut) == 131, "PwStateOut wire size changed");

// Decode the Betaflight arming-disable bitmask to names (4.5.2 order), so the
// pilot can see *why* it won't arm while setting up switches. ARM_SWITCH is
// suppressed — it only ever lights as a consequence of another flag.
static std::string armingBlockReason(uint32_t flags)
{
    static const char* names[] = {
        "NO_GYRO", "FAILSAFE", "RX_FAILSAFE", "NOT_DISARMED", "BOXFAILSAFE",
        "RUNAWAY_TAKEOFF", "CRASH_DETECTED", "THROTTLE", "ANGLE",
        "BOOT_GRACE_TIME", "NOPREARM", "LOAD", "CALIBRATING", "CLI",
        "CMS_MENU", "BST", "MSP", "PARALYZE", "GPS", "RESC", "DSHOT_TELEM",
        "REBOOT_REQUIRED", "DSHOT_BITBANG", "ACC_CALIBRATION",
        "MOTOR_PROTOCOL", "ARM_SWITCH",
    };
    if (flags == 0) return "(ready to arm)";
    std::string out;
    for (int i = 0; i < 25; i++) {  // bits below ARM_SWITCH
        if (flags & (1u << i)) {
            if (!out.empty()) out += ",";
            out += names[i];
        }
    }
    if (out.empty()) {
        // only ARM_SWITCH (bit 25) left: everything else is clear, but the
        // arm switch was already high when arming became allowed
        return "flip ARM switch OFF then ON";
    }
    return out;
}

bool Server::start(uint16_t port)
{
#if defined(_WIN32)
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "[pw][net] WSAStartup failed\n");
        return false;
    }
#endif

    mFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (mFd < 0) {
        perror("[pw][net] socket");
        return false;
    }

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(mFd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[pw][net] bind");
        return false;
    }

    // short recv timeout: when idle (no client), the loop still ticks the
    // firmware so MSP/CLI/Configurator stays responsive (5 ms ~= 200 Hz)
#if defined(_WIN32)
    // Winsock's SO_RCVTIMEO takes a DWORD of milliseconds, not a timeval.
    DWORD tv = 5;
    setsockopt(mFd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#else
    timeval tv {0, 5000};
    setsockopt(mFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

    printf("[pw][net] protocol server on udp:%u\n", port);
    return true;
}

void Server::sendTo(const void* payload, uint16_t len, uint8_t type)
{
    if (!mHaveClient) return;

    uint8_t buf[sizeof(PwHeader) + 512];
    PwHeader hdr { PW_MAGIC, PW_VERSION, type, len };
    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), payload, len);

    sockaddr_in dst {};
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = mClientAddr;
    dst.sin_port = mClientPort;
    // Winsock's sendto takes a `const char*` buffer (POSIX takes const void*,
    // which accepts the cast too).
    sendto(mFd, (const char*)buf, sizeof(hdr) + len, 0, (sockaddr*)&dst, sizeof(dst));
}

/* Debug tooling for the reset-residue hunt (PROPWASH_DUMP_STATE=<path>):
 * write every writable static section of the process to a file right after
 * the first-contact reset. Firmware state in SITL is just process memory, so
 * two runs that behave differently can be diffed byte-for-byte and the
 * offsets resolved to symbols with `nm` — no theorizing.
 *
 * Format: per section, a text line "SECT <seg> <sect> <unslid-vmaddr> <size>"
 * followed by <size> raw bytes. Unslid addresses are recorded so offsets map
 * onto `nm` output regardless of ASLR (tools/tester/state_diff.py consumes
 * this; pointer-valued bytes differ per process via ASLR and self-filter as
 * within-class-unstable there). */
static void dumpStaticState(const char* path)
{
#if defined(__APPLE__)
    FILE* f = fopen(path, "wb");
    if (!f) { perror("[pw][dump] fopen"); return; }

    const struct mach_header_64* mh = &_mh_execute_header;
    intptr_t slide = 0;
    for (uint32_t i = 0; i < _dyld_image_count(); i++) {
        if (_dyld_get_image_header(i) == (const struct mach_header*)mh) {
            slide = _dyld_get_image_vmaddr_slide(i);
            break;
        }
    }

    const struct load_command* lc = (const struct load_command*)(mh + 1);
    for (uint32_t i = 0; i < mh->ncmds; i++) {
        if (lc->cmd == LC_SEGMENT_64) {
            const struct segment_command_64* seg =
                (const struct segment_command_64*)lc;
            if ((seg->initprot & VM_PROT_WRITE)
                && strncmp(seg->segname, "__DATA", 6) == 0) {
                const struct section_64* sec = (const struct section_64*)(seg + 1);
                for (uint32_t s = 0; s < seg->nsects; s++, sec++) {
                    // pure pointer-fixup sections are ASLR noise by definition
                    if (strncmp(sec->sectname, "__got", 5) == 0
                        || strncmp(sec->sectname, "__la_symbol", 11) == 0
                        || strncmp(sec->sectname, "__thread", 8) == 0) {
                        continue;
                    }
                    fprintf(f, "SECT %.16s %.16s 0x%llx %llu\n",
                            seg->segname, sec->sectname,
                            (unsigned long long)sec->addr,
                            (unsigned long long)sec->size);
                    fwrite((const void*)((uintptr_t)sec->addr + slide), 1,
                           (size_t)sec->size, f);
                }
            }
        }
        lc = (const struct load_command*)((const uint8_t*)lc + lc->cmdsize);
    }
    fclose(f);
    printf("[pw][dump] static state -> %s\n", path);
#elif defined(__linux__)
    FILE* f = fopen(path, "wb");
    if (!f) { perror("[pw][dump] fopen"); return; }
    // NOTE: addresses are the RUNTIME ones; with a PIE binary, symbolizing
    // requires subtracting the load base by hand. The macOS path (primary
    // dev machine) records unslid addresses.
    fprintf(f, "SECT __DATA __data %p %zu\n", (void*)__data_start,
            (size_t)(_edata - __data_start));
    fwrite(__data_start, 1, (size_t)(_edata - __data_start), f);
    fprintf(f, "SECT __DATA __bss %p %zu\n", (void*)__bss_start,
            (size_t)(_end - __bss_start));
    fwrite(__bss_start, 1, (size_t)(_end - __bss_start), f);
    fclose(f);
    printf("[pw][dump] static state -> %s\n", path);
#else
    (void)path;
    printf("[pw][dump] not supported on this platform\n");
#endif
}

/* First-contact diagnostics: idle-tick count (the boot-window variable that
 * decides the calibration-progress "coin") and the optional state dump. */
static void firstContactDiagnostics(uint64_t idleTicks)
{
    printf("[pw][net] first contact after %llu idle ticks\n",
           (unsigned long long)idleTicks);
    const char* dumpPath = getenv("PROPWASH_DUMP_STATE");
    if (dumpPath && dumpPath[0]) {
        dumpStaticState(dumpPath);
    }
}

/* Wall-clock milliseconds. Used ONLY to decide whether a client is still
 * driving the sim — never to advance simulated time. */
static uint64_t nowMillis()
{
    return (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

/* How long without a PW_STATE_IN before a client counts as gone, at which point
 * idle-ticking resumes so the Configurator stays alive on an abandoned core.
 *
 * Deliberately generous. In lockstep the client owns the clock, so a client
 * that merely stalls — GC, a slow frame, a loaded CI runner — must not have the
 * gap silently filled in by idle ticks; that reintroduces exactly the
 * wall-clock dependency this mode exists to remove. 250 ms was too tight and
 * made the determinism test flaky under CI load. The cost of erring long is
 * only that MSP takes a couple of seconds to come back after a client exits. */
static constexpr uint64_t CLIENT_IDLE_MS = 3000;

static void applyInit(const PwInit& p, StateInit& s)
{
    for (int i = 0; i < 4; i++) {
        s.motorKV[i] = p.motor_kv[i];
        s.motorR[i]  = p.motor_R[i];
        s.motorI0[i] = p.motor_I0[i];
        s.quadMotorPos[i] = { p.quad_motor_pos[i].x, p.quad_motor_pos[i].y, p.quad_motor_pos[i].z };
    }
    s.propBladeCount   = p.prop_blade_count;
    s.propMaxRpm       = p.prop_max_rpm;
    s.propAFactor      = p.prop_a_factor;
    s.propTorqueFactor = p.prop_torque_factor;
    s.propInertia      = p.prop_inertia;
    s.propThrustFactor = { p.prop_thrust_factor.x, p.prop_thrust_factor.y, p.prop_thrust_factor.z };
    s.frameDragArea    = { p.frame_drag_area.x, p.frame_drag_area.y, p.frame_drag_area.z };
    s.frameDragConstant = p.frame_drag_constant;
    s.quadMass          = p.quad_mass;
    s.quadInvInertia    = { p.quad_inv_inertia.x, p.quad_inv_inertia.y, p.quad_inv_inertia.z };
    s.quadBatCellCount  = p.bat_cell_count;
    s.quadBatCapacity        = p.bat_capacity_mah;
    s.quadBatCapacityCharged = p.bat_capacity_charged_mah;
    s.maxVoltageSag     = p.max_voltage_sag;
    s.minPropWashSpeed      = p.min_propwash_speed;
    s.maxPropWashSpeed      = p.max_propwash_speed;
    s.propWashAngleOfAttack = p.propwash_angle_of_attack;
    s.propWashFactor        = p.propwash_factor;
    s.seed                  = p.seed;   /* was declared on the wire but ignored */
}

void Server::run(SimITL::Sim& sim, const StateInit& defaultInit, Joystick* js)
{
    StateInit initState = defaultInit;
    StateInput input {};
    inputDefaults(input);

    bool booted = false;
    bool rcOverrideActive = false;
    // Lockstep is the protocol's documented default (PW_CMD_LOCKSTEP): the core
    // advances only on PW_STATE_IN. PW_CMD_REALTIME switches to free-running.
    bool lockstep = true;
    uint64_t lastStateInMs = 0;
    uint64_t idleTicks = 0;
    float rcOverride[8] = {0, 0, -1, 0, -1, -1, -1, -1};
    uint64_t stateInCount = 0;

    auto boot = [&]() {
        if (!booted) {
            sim.init(initState);
            BF::configureDefaultModes();
            BF::disableRunawayTakeoff();
            booted = true;
        }
    };

    // Boot the firmware immediately so MSP/CLI (TCP 5761) is up before any
    // client connects — the Configurator and pw_cli.py can attach right away.
    boot();

    uint8_t buf[2048];

    for (;;) {
        sockaddr_in src {};
        socklen_t srcLen = sizeof(src);
        ssize_t n = recvfrom(mFd, (char*)buf, sizeof(buf), 0, (sockaddr*)&src, &srcLen);

        if (n < 0) {
            // Recv timeout. Idle-tick to keep the firmware's scheduler (hence
            // MSP/CLI/Configurator) alive while nothing is driving the sim —
            // without this an idle core leaves the Configurator dead.
            //
            // But ONLY while nothing is driving. This used to fire
            // unconditionally, on the assumption that "when a client is
            // driving, packets arrive faster than the timeout". That is an
            // assumption, not an enforcement: any packet later than 5 ms — GC,
            // scheduler jitter, or the whole client boot window — injected 5 ms
            // of simulated time nobody asked for, into the same accumulator the
            // client drives. Two identical input sequences therefore diverged,
            // which broke the project's central determinism claim.
            //
            // In lockstep mode (the protocol's documented default) a live
            // client is the sole source of time. PW_CMD_REALTIME opts back into
            // free-running for anyone who wants it.
            const bool clientDriving =
                lockstep && stateInCount > 0 &&
                (nowMillis() - lastStateInMs) < CLIENT_IDLE_MS;
            if (!clientDriving) {
                input.delta = 0.005f;
                // `input` outlives the client: without this a departed
                // client's last contact manifold would keep applying forces
                // for as long as the core idle-ticks.
                input.contact = 0;
                input.contactCount = 0;
                sim.update(input);
                idleTicks++;
            }
            continue;
        }
        if (n < (ssize_t)sizeof(PwHeader)) continue;

        PwHeader hdr;
        memcpy(&hdr, buf, sizeof(hdr));
        if (hdr.magic != PW_MAGIC || hdr.version != PW_VERSION) continue;
        if ((ssize_t)(sizeof(PwHeader) + hdr.payload_len) > n) continue;

        mClientAddr = src.sin_addr.s_addr;
        mClientPort = src.sin_port;
        mHaveClient = true;

        const uint8_t* payload = buf + sizeof(PwHeader);

        switch (hdr.type) {

        case PW_INIT: {
            if (hdr.payload_len < sizeof(PwInit)) break;
            PwInit p;
            memcpy(&p, payload, sizeof(p));
            applyInit(p, initState);
            if (booted) {
                sim.reinitPhysics(initState);
            } else {
                boot();
            }

            PwInitAck ack {};
            ack.protocol_version = PW_VERSION;
            snprintf(ack.fc_variant, sizeof(ack.fc_variant), "BTFL");
            snprintf(ack.fc_version, sizeof(ack.fc_version), "4.5.2");
            snprintf(ack.core_version, sizeof(ack.core_version), "propwash-m2");
            sendTo(&ack, sizeof(ack), PW_INIT_ACK);
            break;
        }

        case PW_STATE_IN: {
            if (hdr.payload_len < sizeof(PwStateIn)) break;
            PwStateIn p;
            memcpy(&p, payload, sizeof(p));

            // Wall clock is read here for one purpose only: deciding whether a
            // client is still driving, so the idle tick knows to stay out of
            // the way. It never influences how far the sim advances.
            lastStateInMs = nowMillis();

            // This was never incremented. Everything keyed off it was
            // therefore stuck at zero: the "throttled to ~15 Hz" OSD send
            // below fired on every single frame (0 % 8 == 0), as did the RC
            // heartbeat log.
            stateInCount++;

            if (stateInCount == 1) {
                // First contact. Until now the core has been idle-ticking to
                // keep MSP/CLI alive, and how much simulated time that covered
                // depends on when the client happened to start — which showed
                // up as a different battery state on frame 0 of every run.
                // Discard it so a client session always begins from the same
                // state. This includes re-initialising the firmware: the idle
                // ticks advanced firmware time too, so resetting only the
                // physics left boot-grace and calibration at a different point
                // in every run. Settings are unaffected — BF::init() reloads
                // them from the eeprom, so anything applied and saved over the
                // CLI survives.
                sim.reset(initState);
                firstContactDiagnostics(idleTicks);
            }

            boot();

            input.delta = p.dt;

            // RC priority: local joystick > PW_RC_OVERRIDE > client packet
            if (js && js->isOpen()) {
                js->poll();
                memcpy(input.rcData, js->channels(), sizeof(input.rcData));
            } else if (rcOverrideActive) {
                memcpy(input.rcData, rcOverride, sizeof(input.rcData));
            } else {
                memcpy(input.rcData, p.rc, sizeof(input.rcData));
            }

            input.position = { p.position.x, p.position.y, p.position.z };
            quat q { p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w };
            mat3 basis = quat_to_mat3(q);
            copy(input.rotation, basis);
            input.angularVelocity = { p.angular_velocity.x, p.angular_velocity.y, p.angular_velocity.z };
            input.linearVelocity  = { p.linear_velocity.x, p.linear_velocity.y, p.linear_velocity.z };
            input.gyroBaseNoiseAmp = p.gyro_noise_amp;
            for (int i = 0; i < 4; i++) {
                input.propDamage[i]   = p.prop_damage[i];
                input.groundEffect[i] = p.ground_effect[i];
            }
            input.vbat = p.vbat_charged;
            input.contact = p.contact;

            input.contactCount = p.contact_count > PW_MAX_CONTACTS
                ? PW_MAX_CONTACTS : p.contact_count;
            for (int i = 0; i < input.contactCount; i++) {
                input.contacts[i].pointBody   = { p.contacts[i].point_body.x,
                                                  p.contacts[i].point_body.y,
                                                  p.contacts[i].point_body.z };
                input.contacts[i].normalWorld = { p.contacts[i].normal_world.x,
                                                  p.contacts[i].normal_world.y,
                                                  p.contacts[i].normal_world.z };
                input.contacts[i].depth   = p.contacts[i].depth;
                input.contacts[i].surface = p.contacts[i].surface;
            }

            sim.update(input);

            const SimState& st = sim.getSimState();
            const StateOutput& out = sim.getStateUpdate();

            PwStateOut o {};
            o.frame_id = p.frame_id;
            o.sim_time_us = st.microsPassed;
            o.orientation = { out.orientation.w, out.orientation.x, out.orientation.y, out.orientation.z };
            o.angular_velocity = { out.angularVelocity.x, out.angularVelocity.y, out.angularVelocity.z };
            o.linear_velocity  = { out.linearVelocity.x, out.linearVelocity.y, out.linearVelocity.z };
            o.position = p.position; // client-owned; echoed
            o.acceleration = { st.acceleration[0], st.acceleration[1], st.acceleration[2] };
            for (int i = 0; i < 4; i++) {
                o.motor_rpm[i] = out.motorRpm[i];
                o.motor_status[i] = (uint8_t)out.motorStatus[i];
            }
            o.armed = st.armed ? 1 : 0;
            o.arming_disable_flags = (uint32_t)st.armingDisabledFlags;
            o.flight_mode_flags = (uint32_t)st.flightModeFlags;
            o.beeper = st.beep ? 1 : 0;
            o.vbat = st.batteryState.batVoltageSag;
            o.amperage = (float)st.batteryState.amperage;
            for (int i = 0; i < 4; i++) o.prop_damage[i] = out.propDamage[i];
            o.crash_flags = out.crashFlags;
            sendTo(&o, sizeof(o), PW_STATE_OUT);

            // OSD grid, throttled to ~15 Hz (it changes far slower than the
            // physics tick and is a full 480-byte packet)
            if (stateInCount % 8 == 0) {
                PwOsd osd {};
                memcpy(osd.chars, out.osd, sizeof(osd.chars));
                sendTo(&osd, sizeof(osd), PW_OSD);
            }

            // Log the 8 RC channels + decoded arm-block reason whenever the
            // armed state or disable flags change (so flipping a switch gives
            // immediate feedback), plus a periodic heartbeat.
            static uint32_t lastFlags = 0xFFFFFFFFu;
            static uint8_t lastArmed = 0xFF;
            if (o.arming_disable_flags != lastFlags || o.armed != lastArmed
                || stateInCount % 1024 == 0) {
                lastFlags = o.arming_disable_flags;
                lastArmed = o.armed;
                printf("[pw][rc] AETR=%.2f/%.2f/%.2f/%.2f  ch5-8=%.2f/%.2f/%.2f/%.2f  "
                       "armed=%d  block:%s\n",
                       input.rcData[0], input.rcData[1], input.rcData[2], input.rcData[3],
                       input.rcData[4], input.rcData[5], input.rcData[6], input.rcData[7],
                       o.armed, armingBlockReason(o.arming_disable_flags).c_str());
            }
            break;
        }

        case PW_MOTOR_IN: {
            if (hdr.payload_len < sizeof(PwMotorIn)) break;
            PwMotorIn p;
            memcpy(&p, payload, sizeof(p));

            lastStateInMs = nowMillis();
            stateInCount++;
            if (stateInCount == 1) {
                sim.reset(initState);
                firstContactDiagnostics(idleTicks);
            }
            boot();

            input.delta = p.dt;
            // No RC: the firmware is bypassed. Sticks stay safe/neutral so that
            // if anything does read rcData it sees a disarmed idle.
            static const float safeRc[8] = {0, 0, -1, 0, -1, -1, -1, -1};
            memcpy(input.rcData, safeRc, sizeof(input.rcData));

            input.position = { p.position.x, p.position.y, p.position.z };
            quat q { p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w };
            mat3 basis = quat_to_mat3(q);
            copy(input.rotation, basis);
            input.angularVelocity = { 0, 0, 0 };
            input.linearVelocity  = { 0, 0, 0 };
            input.gyroBaseNoiseAmp = 0.0f;   // deterministic replay by default
            for (int i = 0; i < 4; i++) {
                input.propDamage[i]   = 0.0f;
                input.groundEffect[i] = 0.0f;
            }
            input.vbat = 16.8f;              // full 4S; physics models the sag
            input.contact = p.contact_count ? 1 : 0;
            input.contactCount = p.contact_count > PW_MAX_CONTACTS
                ? PW_MAX_CONTACTS : p.contact_count;
            for (int i = 0; i < input.contactCount; i++) {
                input.contacts[i].pointBody   = { p.contacts[i].point_body.x,
                                                  p.contacts[i].point_body.y,
                                                  p.contacts[i].point_body.z };
                input.contacts[i].normalWorld = { p.contacts[i].normal_world.x,
                                                  p.contacts[i].normal_world.y,
                                                  p.contacts[i].normal_world.z };
                input.contacts[i].depth   = p.contacts[i].depth;
                input.contacts[i].surface = p.contacts[i].surface;
            }

            float motor[4] = { p.motor[0], p.motor[1], p.motor[2], p.motor[3] };
            sim.updateMotors(input, motor);

            const SimState& st = sim.getSimState();
            const StateOutput& out = sim.getStateUpdate();

            PwStateOut o {};
            o.frame_id = p.frame_id;
            o.sim_time_us = st.microsPassed;
            o.orientation = { out.orientation.w, out.orientation.x, out.orientation.y, out.orientation.z };
            o.angular_velocity = { out.angularVelocity.x, out.angularVelocity.y, out.angularVelocity.z };
            o.linear_velocity  = { out.linearVelocity.x, out.linearVelocity.y, out.linearVelocity.z };
            o.position = p.position; // client-owned; echoed
            o.acceleration = { st.acceleration[0], st.acceleration[1], st.acceleration[2] };
            for (int i = 0; i < 4; i++) {
                o.motor_rpm[i] = out.motorRpm[i];
                o.motor_status[i] = (uint8_t)out.motorStatus[i];
            }
            o.armed = 0;                 // firmware bypassed
            o.arming_disable_flags = 0;
            o.flight_mode_flags = 0;
            o.beeper = 0;
            o.vbat = st.batteryState.batVoltageSag;
            o.amperage = (float)st.batteryState.amperage;
            for (int i = 0; i < 4; i++) o.prop_damage[i] = out.propDamage[i];
            o.crash_flags = out.crashFlags;
            sendTo(&o, sizeof(o), PW_STATE_OUT);
            break;
        }

        case PW_RC_OVERRIDE: {
            if (hdr.payload_len < sizeof(PwRcOverride)) break;
            PwRcOverride p;
            memcpy(&p, payload, sizeof(p));
            memcpy(rcOverride, p.rc, sizeof(rcOverride));
            rcOverrideActive = true;
            break;
        }

        case PW_COMMAND: {
            if (hdr.payload_len < sizeof(PwCommand)) break;
            PwCommand p;
            memcpy(&p, payload, sizeof(p));
            switch (p.command) {
            case PW_CMD_RESET:
                printf("[pw][net] RESET\n");
                sim.reset(initState);
                rcOverrideActive = false;
                break;
            case PW_CMD_REPAIR:
                // Clear accumulated damage + recharge; pose/firmware untouched
                // (the client owns pose and rights the quad itself).
                printf("[pw][net] REPAIR\n");
                sim.command(CommandType::Repair);
                break;
            case PW_CMD_LOCKSTEP:
                // The protocol's documented default: time advances only on
                // PW_STATE_IN, which is what makes runs reproducible.
                lockstep = true;
                printf("[pw][net] LOCKSTEP\n");
                break;
            case PW_CMD_REALTIME:
                // Free-running: the core self-ticks on the recv timeout even
                // while a client is connected. Reproducibility is forfeit.
                lockstep = false;
                printf("[pw][net] REALTIME (sim is no longer reproducible)\n");
                break;
            default:
                break;
            }
            break;
        }

        default:
            break;
        }
    }
}

} // namespace pw
