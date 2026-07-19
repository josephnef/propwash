#ifndef SIMITL_STATE_H
#define SIMITL_STATE_H
#include "util/vector_math.h"
#include "util/LowPassFilter.h"
#include "sim_packets.h"

#include <array>
#include <cstdint>


namespace SimITL{

  // copy helper
  inline void copy(vec3& out, const Vec3F& in){
    out[0] = in.x;
    out[1] = in.y;
    out[2] = in.z;
  }

  inline void copy(mat3& out, const Vec3F* in){
    for(int i = 0; i < 3; i++){
      copy(out[i], in[i]);
    }
  }

  inline void copy(Vec3F& out, const vec3& in){
    out.x = in[0];
    out.y = in[1];
    out.z = in[2];
  }

  inline void copy(Vec3F* out, const mat3& in){
    for(int i = 0; i < 3; i++){
      copy(out[i], in[i]);
    }
  }

  // per motor realtime state
  struct MotorState {
    vec3 position = {0.0f, 0.0f, 0.0f};
    // pwm signal in percent [0,1]
    float pwm = 0.0f;
    //low pass filtered pwm value
    LowPassFilter pwmLowPassFilter{};
    // motor core temp in deg C
    float temp = 0.0f;
    // current running through motor in Amps
    float current = 0.0f;
    // motor revolutions per minute
    float rpm = 0.0f;
    // thrust output of motor / propeller combo
    float thrust = 0.0f;
    // motor torque
    float mTorque = 0.0f;
    // propeller torque, counter acting motor torque
    float pTorque = 0.0f;

    // low pass filtered prop wash
    LowPassFilter propWashLowPassFilter{};

    // sinusoidal phase of the motor rotation used for noise simulation
    float phase = 0.0f;
    // phase freq * 2
    float phaseHarmonic1 = 0.0f;
    // phase freq * 3
    float phaseHarmonic2 = 0.0f;
    // phase freq * 0.01f
    float phaseSlow = 0.0f;

    // motor status flags
    // \see MotorStatus
    int32_t status = 0;

    // spin direction commanded over virtual DSHOT: +1 normal, -1 reversed
    // (crashflip). Thrust flips sign and pays a reverse-efficiency penalty.
    int8_t spinDir = 1;
  };

  struct BatteryState {
    // current battery voltage
    float batVoltage    = 0.0f; // in V
    // current sagged battery voltage
    float batVoltageSag = 0.0f; // in V
    // current battery capacity
    double batCapacity  = 0.0f; // in mAh

    // current amp draw in amps
    double amperage = 0.0f;
    // current mAh drawn from battery
    double mAhDrawn = 0.0f;
  };

  /* Impact-damage tuning. Every threshold is a SPEED (m/s), not a force —
   * independent of the contact-spring stiffness (retuning the solver must
   * not retune damage) and immune to resting-load false positives by
   * construction: a parked quad has v_n ~ 0 no matter how hard it presses.
   * Calibrated for the 0.33 kg CineLog35 (terminal velocity ~17 m/s, so a
   * long fall always lands far above vCrash). */
  struct DamageConfig {
    float vSafe            = 1.8f;    // m/s: below this, zero damage
    float vCrash           = 7.0f;    // m/s (surface-scaled): structural crash
    float impactGain       = 0.35f;   // frame damage at the vCrash end of the ramp
    float propStrikeVMin   = 0.7f;    // m/s: a spinning prop chips on any touch
    float propStrikeBase   = 0.25f;   // instant damage floor for a prop strike
    float propStrikeGain   = 0.50f;   // + gain * min(1, v_n / 8)
    float propStrikeRadius = 0.05f;   // m: xz distance to a motor axis
    float propStrikeYBand  = 0.03f;   // m: |y - motor plane| gate
    float propSpinMinRpm   = 3000.0f; // below this a "strike" is just a frame tap
    float strikeTorque     = 0.8f;    // N*m yaw kick from a prop strike
    float strikeTorqueS    = 0.010f;  // s: kick duration (~8.8 rad/s on I_yaw)
    float strikeRpmKeep    = 0.3f;    // rpm multiplier applied once on strike
    float crashDamage      = 0.9f;    // per-motor damage from a structural crash
                                      // (max thrust 1.12 N < 3.24 N weight:
                                      // hover becomes impossible via physics)
    float damagedStatusAt  = 0.25f;   // eff damage that latches MotorDamaged
    float motorReachXZ     = 0.11f;   // m: frame-damage proximity falloff
    float distribFloor     = 0.25f;   // every impact stresses the whole frame
    float cooldownS        = 0.25f;   // s: per-motor re-damage lockout
                                      // (sustained scraping re-damages at 4 Hz)
    // index = PwSurfaceType: GROUND (dirt), GATE (plastic), TREE, OBJECT
    float surfaceFactor[4] = {0.7f, 0.5f, 0.85f, 1.0f};
  };

  struct DamageState {
    float accumulated[4] {};    // core-owned impact damage, 0..1
    float eff[4] {};            // max(client input, accumulated) — what physics uses
    float cooldown[4] {};       // s remaining per motor
    int   strikeTicks = 0;      // remaining yaw-kick sub-ticks
    float strikeYawSign = 0.0f; // summed -motor_dir of struck motors
    bool  crashed = false;      // structural-crash latch (REPAIR/RESET clears)
  };

  /* One core-resolved contact. Loaded from the client's manifold once per
   * frame, then force-integrated every 50 us sub-tick — the penetration depth
   * evolves with the contact-point velocity until the client's next manifold
   * replaces the set or the point separates (depth <= 0). */
  struct ActiveContact {
    bool  active = false;
    vec3  pointBody {0.0f, 0.0f, 0.0f};   // body frame, m
    vec3  normalWorld {0.0f, 1.0f, 0.0f}; // unit, surface -> quad
    float depth = 0.0f;                   // m
    uint8_t surface = 0;                  // PwSurfaceType
    // per-sub-tick scratch
    vec3  rWorld {0.0f, 0.0f, 0.0f};      // lever arm R*pointBody
    vec3  force {0.0f, 0.0f, 0.0f};       // last applied force, N
  };

  /**
   * \brief Stores the state of the simulation including incoming and 
   * outgoing network packets.
   */
  struct SimState {
    // initial quad/physics params
    StateInit stateInit {};

    // current internal state
    StateInput stateInput {};

    // outgoing packets
    StateOutput stateOutput {};

    bool armed = false;
    int armingDisabledFlags = 0;
    int flightModeFlags = 0;

    bool beep = false;

    // rc data
    uint16_t rcData[16] {};
    uint32_t rcDataReceptionTimeUs = 0U;

    std::array<MotorState, 4> motorsState {};
    BatteryState batteryState {};

    // contact manifold being resolved (mirrors StateInput.contacts)
    std::array<ActiveContact, 6> contacts {};
    int contactCount = 0;
    // per-client-frame contact statistics (damage-model inputs)
    float contactPeakForceN = 0.0f; // max |F| over the frame's sub-ticks
    float contactImpulseNs = 0.0f;  // sum |F|*dt over the frame

    DamageConfig damageCfg {};
    DamageState damage {};

    // current wind velocity, world frame (updated per tick from StateInit)
    vec3 wind {0, 0, 0};

    // firmware crash state, read back each tick (crash_flags bits 1/2)
    bool bfCrashRecoveryActive = false;
    bool bfFlipOverActive = false;

    vec3 acceleration{0, 0, 0};

    vec3 gyroNoise{0, 0, 0};
    vec3 motorNoise{0, 0, 0};
    vec3 combinedNoise{0, 0, 0};

    LowPassFilter gyroLowPassFilter[3]{};

    // gyro / acc
    quat rotation{1, 0, 0, 1};
    vec3 gyro{0, 0, 0};
    vec3 acc{0, 0, 0};

    float frameHarmonicPhase1 = 0.0f;
    float frameHarmonicPhase2 = 0.0f;

    // time passed in micro seconds
    uint64_t microsPassed = 0;

    /* Noise RNG state. Deliberately owned by the sim rather than libc: rand()
     * is process-global, cannot be reset, and is shared with any other caller,
     * so a single extra draw anywhere shifts the whole stream and every
     * subsequent trajectory with it. Seeded from PwInit.seed. */
    uint32_t rngState = 0x9E3779B9u;
  };

} // SIMITL_STATE_H

#endif