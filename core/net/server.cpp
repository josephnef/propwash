#include "server.h"

#include <cstdio>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "sim/profile_cinelog35.h"
#include "sim/util/vector_math.h"

namespace pw {

using namespace SimITL;

bool Server::start(uint16_t port)
{
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

    // periodic timeout so the process stays responsive
    timeval tv {1, 0};
    setsockopt(mFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

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
    sendto(mFd, buf, sizeof(hdr) + len, 0, (sockaddr*)&dst, sizeof(dst));
}

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
}

void Server::run(SimITL::Sim& sim, const StateInit& defaultInit, Joystick* js)
{
    StateInit initState = defaultInit;
    StateInput input {};
    inputDefaults(input);

    bool booted = false;
    bool rcOverrideActive = false;
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

    uint8_t buf[2048];

    for (;;) {
        sockaddr_in src {};
        socklen_t srcLen = sizeof(src);
        ssize_t n = recvfrom(mFd, buf, sizeof(buf), 0, (sockaddr*)&src, &srcLen);
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
            sendTo(&o, sizeof(o), PW_STATE_OUT);

            if (++stateInCount % 1024 == 0) {
                printf("[pw][net] frames=%llu armed=%d dis=0x%x rc_src=%s\n",
                       (unsigned long long)stateInCount, o.armed,
                       o.arming_disable_flags,
                       (js && js->isOpen()) ? "joystick" : (rcOverrideActive ? "override" : "client"));
            }
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
            if (p.command == PW_CMD_RESET) {
                printf("[pw][net] RESET\n");
                sim.reinitPhysics(initState);
                rcOverrideActive = false;
            }
            break;
        }

        default:
            break;
        }
    }
}

} // namespace pw
