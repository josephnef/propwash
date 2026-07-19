#include "physics.h"
#include <cstdio>
#include "util/SimplexNoise.h"
#include "contact_materials.h"
#include "bf.h"
#include <algorithm>
#include <cassert>
#include <cstring>

namespace SimITL{
  #ifndef M_PI
  #define M_PI 3.14159265358979
  #endif

  const auto AIR_RHO = 1.225f;

  /* xorshift32 over sim-owned state. Replaces libc rand(), which was never
   * seeded (so it happened to repeat across processes) but is process-global
   * and un-resettable — RESET could not restore it, and any unrelated rand()
   * caller would shift every subsequent draw. Determinism is the project's
   * headline claim, so the noise source has to belong to the sim. */
  inline uint32_t nextRand(uint32_t& s){
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }

  // -1.0 , 1.0
  inline float randf(uint32_t& s){
    return (static_cast<float>(nextRand(s)) / static_cast<float>(0xFFFFFFFFu)) * 2.0f - 1.0f;
  }

  inline float rpmToHz(float rpm){
    return rpm / 60.0f;
  }

  static int s_update_err_cnt = 0;

  bool Physics::checkSimState(){
    if(mSimState == nullptr){
      if(s_update_err_cnt % 2000 == 0){
        printf("Physics::update failed, no SimState set.\n");
      }
      s_update_err_cnt++;
      return false;
    }
    return true;
  }

  void Physics::setSimState(SimState* state){
    mSimState = state;
  }

  void Physics::initState(const StateInit& state){
    //copy data
    memcpy( (void *)&(mSimState->stateInit), (const void *)(&state), sizeof(StateInit) );

    // Reseed the noise RNG. Without this a reset restored the physics but not
    // the noise stream, so a "reset" run diverged from a fresh one.
    mSimState->rngState = mSimState->stateInit.seed ? mSimState->stateInit.seed
                                                    : 0x9E3779B9u;

    // is calculated now
    mSimState->batteryState.batVoltage = 0.1f;
    mSimState->batteryState.batVoltageSag = 0.1f;
    mSimState->batteryState.batCapacity = mSimState->stateInit.quadBatCapacityCharged;

    for (auto i = 0u; i < 4; i++) {
      mSimState->motorsState[i].position[0] = mSimState->stateInit.quadMotorPos[i].x;
      mSimState->motorsState[i].position[1] = mSimState->stateInit.quadMotorPos[i].y;
      mSimState->motorsState[i].position[2] = mSimState->stateInit.quadMotorPos[i].z;
      mSimState->motorsState[i].temp        = mSimState->stateInit.ambientTemp;
    }
  }

  void Physics::updateState(const StateInput& state){
    BF::setDebugValue(E_DEBUG_SIM, 2, state.contact * 1000);

    /* Velocities are core-authoritative: contacts arrive as manifold points
     * and are resolved as *forces* in calculatePhysics, so the firmware's
     * virtual accel/gyro see impacts and ground support the way the real
     * sensors would (a velocity edit is invisible to the accelerometer).
     * The client's velocity fields are its echo of the last output. */
    vec3 coreAngularVelocity;
    copy(coreAngularVelocity, mSimState->stateInput.angularVelocity);
    vec3 coreLinearVelocity;
    copy(coreLinearVelocity, mSimState->stateInput.linearVelocity);

    //copy data (client pose is authoritative; velocities restored below)
    memcpy( (void *)&(mSimState->stateInput), (const void *)(&state), sizeof(StateInput) );

    copy(mSimState->stateInput.angularVelocity, coreAngularVelocity);
    copy(mSimState->stateInput.linearVelocity, coreLinearVelocity);

    // load this frame's contact manifold; it replaces the previous set and
    // is then evolved per sub-tick until the next frame
    auto& contacts = mSimState->contacts;
    const std::array<ActiveContact, 6> prev = contacts;
    const int n = std::min<int>(state.contactCount, (int)contacts.size());
    for (int i = 0; i < n; i++) {
      auto& c = contacts[i];
      c.active = true;
      copy(c.pointBody, state.contacts[i].pointBody);
      copy(c.normalWorld, state.contacts[i].normalWorld);
      c.normalWorld = normalize(c.normalWorld); // degenerate -> zero force
      c.depth = clamp(state.contacts[i].depth, 0.0f, CONTACT_DMAX);
      c.surface = state.contacts[i].surface;
      c.rWorld = {0.0f, 0.0f, 0.0f};
      c.force = {0.0f, 0.0f, 0.0f};

      /* Depth continuity. The client integrates position with one
       * end-of-frame velocity (rectangle rule) while the core evolved this
       * depth per sub-tick, so the client's geometric depth carries an
       * O(a*dt^2) error every frame — enough to differentially pump a
       * rocking mode when resting on an asymmetric contact set. For a
       * persisting contact keep the core's evolved depth; the client
       * re-anchors only when geometry genuinely changed (> 2 mm disagreement,
       * far above the ~0.1 mm static penetration and the sub-mm frame
       * integration error). */
      for (const auto& p : prev) {
        if (!p.active || p.surface != c.surface) continue;
        const vec3 d = c.pointBody - p.pointBody;
        if (length2(d) < 0.02f * 0.02f) {
          if (std::fabs(p.depth - c.depth) < 0.002f) {
            c.depth = p.depth;
          }
          break;
        }
      }
    }
    for (size_t i = n; i < contacts.size(); i++) {
      contacts[i].active = false;
    }
    mSimState->contactCount = n;
    mSimState->contactPeakForceN = 0.0f;
    mSimState->contactImpulseNs = 0.0f;
  }

  void Physics::updateGyro(double dt){
    updateGyroNoise(mSimState->stateInput, mSimState->gyroNoise);
    updateMotorNoise(dt, mSimState->stateInput, mSimState->motorNoise);
    mSimState->combinedNoise = mSimState->gyroNoise + mSimState->motorNoise;

    // calc gyro and acc output
    mat3 basis;
    copy(basis, mSimState->stateInput.rotation);
    mSimState->rotation = mat3_to_quat(basis);

    vec3 angularVelocity;
    copy(angularVelocity, mSimState->stateInput.angularVelocity);
    angularVelocity = angularVelocity + mSimState->combinedNoise;

    constexpr float cutoffFreq = 300.0f;
    angularVelocity[0] = mSimState->gyroLowPassFilter[0].update(angularVelocity[0], dt, cutoffFreq);
    angularVelocity[1] = mSimState->gyroLowPassFilter[1].update(angularVelocity[1], dt, cutoffFreq);
    angularVelocity[2] = mSimState->gyroLowPassFilter[2].update(angularVelocity[2], dt, cutoffFreq);

    mSimState->gyro = xform_inv(basis, angularVelocity);

    //todo: validate and fix gyro attitude setup.
    auto gravity_acceleration = vec3{0, -9.81f, 0};
    mSimState->acc = xform_inv(basis, mSimState->acceleration + gravity_acceleration);
  }

  void Physics::updatePhysics(double dt){
    updateWind();
    updateDamage(dt);
    float motorsTorque = calculateMotors(dt, mSimState->stateInput, mSimState->motorsState);
    mSimState->acceleration = calculatePhysics(dt, mSimState->stateInput, mSimState->motorsState, motorsTorque);

    // update battery data
    updateBat(dt);

    // prepare update packet
    mat3 basis;
    copy(basis, mSimState->stateInput.rotation);
    quat orientation = mat3_to_quat(basis);
    mSimState->stateOutput.orientation.w = orientation[3];
    mSimState->stateOutput.orientation.x = orientation[0];
    mSimState->stateOutput.orientation.y = orientation[1];
    mSimState->stateOutput.orientation.z = orientation[2];
    mSimState->stateOutput.angularVelocity = mSimState->stateInput.angularVelocity;
    mSimState->stateOutput.linearVelocity = mSimState->stateInput.linearVelocity;

    mSimState->stateOutput.motorRpm[0] = mSimState->motorsState[0].rpm;
    mSimState->stateOutput.motorRpm[1] = mSimState->motorsState[1].rpm;
    mSimState->stateOutput.motorRpm[2] = mSimState->motorsState[2].rpm;
    mSimState->stateOutput.motorRpm[3] = mSimState->motorsState[3].rpm;

    mSimState->stateOutput.motorT[0] = mSimState->motorsState[0].temp;
    mSimState->stateOutput.motorT[1] = mSimState->motorsState[1].temp;
    mSimState->stateOutput.motorT[2] = mSimState->motorsState[2].temp;
    mSimState->stateOutput.motorT[3] = mSimState->motorsState[3].temp;

    mSimState->stateOutput.motorStatus[0] = mSimState->motorsState[0].status;
    mSimState->stateOutput.motorStatus[1] = mSimState->motorsState[1].status;
    mSimState->stateOutput.motorStatus[2] = mSimState->motorsState[2].status;
    mSimState->stateOutput.motorStatus[3] = mSimState->motorsState[3].status;

    for (int i = 0; i < 4; i++) {
      mSimState->stateOutput.propDamage[i] = mSimState->damage.eff[i];
    }
    mSimState->stateOutput.crashFlags =
        (mSimState->damage.crashed ? 1U : 0U) |
        (mSimState->bfCrashRecoveryActive ? 2U : 0U) |
        (mSimState->bfFlipOverActive ? 4U : 0U);
  }

  void Physics::updateWind(){
    const auto& si = mSimState->stateInit;
    if (si.windGustAmp <= 0.0f && si.windMean.x == 0.0f &&
        si.windMean.y == 0.0f && si.windMean.z == 0.0f) {
      // calm: skip entirely so no-wind trajectories stay bit-identical
      mSimState->wind = {0.0f, 0.0f, 0.0f};
      return;
    }
    vec3 wind;
    copy(wind, si.windMean);
    if (si.windGustAmp > 0.0f) {
      const float t = (float)((double)mSimState->microsPassed * 1e-6) * si.windGustFreq;
      const float ph = (float)(si.seed & 0xFFFFu) * 0.001f;
      // simplex is a pure function of its argument — deterministic without
      // touching the noise RNG stream. Vertical gusts weaker: real gust
      // spectra are mostly horizontal.
      wind[0] += SimplexNoise::noise(t + ph + 17.3f) * si.windGustAmp;
      wind[1] += SimplexNoise::noise(t + ph + 29.7f) * si.windGustAmp * 0.3f;
      wind[2] += SimplexNoise::noise(t + ph + 43.1f) * si.windGustAmp;
    }
    mSimState->wind = wind;
  }

  void Physics::updateDamage(double dt){
    auto& dmg = mSimState->damage;
    const auto& cfg = mSimState->damageCfg;

    for (int i = 0; i < 4; i++) {
      dmg.cooldown[i] = std::max(0.0f, dmg.cooldown[i] - (float)dt);
    }
    if (dmg.strikeTicks > 0) {
      dmg.strikeTicks--;
      if (dmg.strikeTicks == 0) {
        dmg.strikeYawSign = 0.0f;
      }
    }

    if (mSimState->contactCount > 0) {
      mat3 rotation;
      copy(rotation, mSimState->stateInput.rotation);
      vec3 linVel;
      copy(linVel, mSimState->stateInput.linearVelocity);
      vec3 angVel;
      copy(angVel, mSimState->stateInput.angularVelocity);
      // must match calculateMotors' motor_dir (yaw reaction sign per motor)
      const float motor_dir[4] = {-1.0f, 1.0f, 1.0f, -1.0f};

      for (const auto& c : mSimState->contacts) {
        if (!c.active) continue;
        const vec3 rW = xform(rotation, c.pointBody);
        const vec3 vPoint = linVel + cross(angVel, rW);
        // approach speed at the contact point; a parked quad has vN ~ 0
        const float vN = std::max(0.0f, -dot(vPoint, c.normalWorld));
        const float S = cfg.surfaceFactor[c.surface < 4 ? c.surface : 3];

        // structural crash: the whole airframe is done, regardless of where
        // it hit — all motors heavily damaged, latch reported to the client
        if (vN * S >= cfg.vCrash) {
          for (int i = 0; i < 4; i++) {
            dmg.accumulated[i] = std::max(dmg.accumulated[i], cfg.crashDamage);
            dmg.cooldown[i] = cfg.cooldownS;
          }
          dmg.crashed = true;
          continue;
        }
        if (vN < cfg.propStrikeVMin) continue;

        // prop strike: contact inside a spinning prop disc — instant heavy
        // damage on that motor, rpm slashed, and a yaw kick (the prop grabs)
        bool struck = false;
        for (int i = 0; i < 4; i++) {
          const Vec3F& mp = mSimState->stateInit.quadMotorPos[i];
          const float dx = c.pointBody[0] - mp.x;
          const float dz = c.pointBody[2] - mp.z;
          if (dx * dx + dz * dz > cfg.propStrikeRadius * cfg.propStrikeRadius) continue;
          if (std::fabs(c.pointBody[1] - mp.y) > cfg.propStrikeYBand) continue;
          if (mSimState->motorsState[i].rpm < cfg.propSpinMinRpm) continue;
          if (dmg.cooldown[i] > 0.0f) continue;
          const float inc = (cfg.propStrikeBase +
                             cfg.propStrikeGain * std::min(1.0f, vN / 8.0f)) * S;
          dmg.accumulated[i] = std::min(1.0f, dmg.accumulated[i] + inc);
          mSimState->motorsState[i].rpm *= cfg.strikeRpmKeep;
          dmg.strikeTicks = (int)(cfg.strikeTorqueS / dt);
          dmg.strikeYawSign += -motor_dir[i];
          dmg.cooldown[i] = cfg.cooldownS;
          struck = true;
        }

        // frame impact: cumulative, quadratic in speed above the safe
        // threshold, proximity-weighted toward the motors nearest the hit
        if (!struck && vN >= cfg.vSafe) {
          const float ramp = (std::min(vN, cfg.vCrash) - cfg.vSafe) /
                             (cfg.vCrash - cfg.vSafe);
          const float inc = cfg.impactGain * S * ramp * ramp;
          for (int i = 0; i < 4; i++) {
            if (dmg.cooldown[i] > 0.0f) continue;
            const Vec3F& mp = mSimState->stateInit.quadMotorPos[i];
            const float dx = c.pointBody[0] - mp.x;
            const float dz = c.pointBody[2] - mp.z;
            const float dist = std::sqrt(dx * dx + dz * dz);
            const float w = std::max(clamp(1.0f - dist / cfg.motorReachXZ, 0.0f, 1.0f),
                                     cfg.distribFloor);
            dmg.accumulated[i] = std::min(1.0f, dmg.accumulated[i] + inc * w);
            dmg.cooldown[i] = cfg.cooldownS;
          }
        }
      }
    }

    // effective damage: the client's scripted input still works, the core's
    // accumulated impact damage layers on top
    for (int i = 0; i < 4; i++) {
      dmg.eff[i] = clamp(std::max(mSimState->stateInput.propDamage[i],
                                  dmg.accumulated[i]), 0.0f, 1.0f);
      if (dmg.eff[i] >= cfg.damagedStatusAt) {
        mSimState->motorsState[i].status |= MotorStatus::MotorDamaged;
      }
    }
  }

  void Physics::updateRotation(double dt, StateInput& state) {
    vec3 angularVelocity;
    copy(angularVelocity, state.angularVelocity);

    const auto w = angularVelocity * dt;
    const mat3 W = {
      vec3{    1, -w[2],  w[1]},
      vec3{ w[2],     1, -w[0]},
      vec3{-w[1],  w[0],     1}
    };

    mat3 rotation;
    copy(rotation, state.rotation);
    rotation = W * rotation;

    /* The first-order update above never re-orthonormalizes, and contact
     * torques are the first thing in this sim that spins the matrix hard
     * between the client's per-frame quaternion echoes (which used to be the
     * only renormalization). Gram-Schmidt the body axes (columns) so R stays
     * in SO(3) even through a sustained max-rate tumble. */
    const vec3 bx = normalize(get_axis(rotation, 0));
    vec3 by = get_axis(rotation, 1);
    by = normalize(by - bx * dot(bx, by));
    const vec3 bz = cross(bx, by);
    for (int i = 0; i < 3; i++) {
      rotation[i][0] = bx[i];
      rotation[i][1] = by[i];
      rotation[i][2] = bz[i];
    }

    copy(state.rotation, rotation);
  }

  /**
   * See: https://things-in-motion.blogspot.com/2018/12/how-to-estimate-torque-of-bldc-pmsm.html
   */
  float Physics::motorCurrent(float motorTorque, float kV){
    return motorTorque * kV / 8.3f;
  }

  /**
   * Calculates motor torque in Nm
   * 
   * param[in] volts Volts supplied by esc based on pwm. Basically vBat * pwm factor.
   * param[in] rpm Current motor rpm.
   * param[in] kV Motor's velocity constant in rpm / V
   * param[in] R Motor's resistance in ohm.
   * param[in] I0 Motor's initial current needed to spin it in A.
   */
  float Physics::motorTorque(float volts, float rpm, float kV, float R, float I0) {
    const auto backEmfV = rpm / std::max(kV, 0.0001f);
    auto current = (volts - backEmfV) / std::max(R, 0.0001f);

    if (current > 0)
        current = std::max(0.0f, current - I0);
    else if (current < 0)
        current = std::min(0.0f, current + I0);


    // Nm per A
    const float NmPerA = 8.3f / std::max(kV, 0.0001f);
    return current * NmPerA;

    // old version
    //return current * 60 / (std::max(kV, 0.0001f) * 2.0f * float(M_PI));
  }

  float Physics::propThrust(float rpm, float vel) {
    // max thrust vs velocity:
    auto propF = mSimState->stateInit.propThrustFactor.x * vel * vel +
                 mSimState->stateInit.propThrustFactor.y * vel +
                 mSimState->stateInit.propThrustFactor.z;

    const auto max_rpm = std::max(mSimState->stateInit.propMaxRpm, 0.01f);
    const auto prop_a = mSimState->stateInit.propAFactor;
    propF = std::max(0.0f, propF);

    // thrust vs rpm (and max thrust) 
    const auto b = (propF - prop_a * max_rpm * max_rpm ) / max_rpm;
    const auto result = b * rpm + prop_a * rpm * rpm;

    return std::max(result, 0.0f);
  }

  float Physics::propTorque(float rpm, float vel) {
    return propThrust(rpm, vel) * mSimState->stateInit.propTorqueFactor;
  }

  void Physics::updateBat(double dt) {
    const double batCapacityFull = std::max(mSimState->stateInit.quadBatCapacity, 1.0f);
    mSimState->batteryState.batVoltage = mBatVoltageCurve.sample(1.0f - (mSimState->batteryState.batCapacity / batCapacityFull)) * mSimState->stateInit.quadBatCellCount;
    mSimState->batteryState.batVoltage = std::max(mSimState->batteryState.batVoltage, 0.1f);

    float pwmSum = 0.0f;
    float rpmSum = 0.0f;
    for(int i = 0; i < 4; i++){
      pwmSum += mSimState->motorsState[i].pwm;
      rpmSum += mSimState->motorsState[i].rpm;
    }
    
    const float powerFactor = std::max(0.0f, pwmSum / 4.0f);
    const float powerFactor2 = powerFactor * powerFactor;
    const float chargeFactorInv = 1.0f - (static_cast<float>(mSimState->batteryState.batCapacity) / 
                                  std::max(mSimState->stateInit.quadBatCapacityCharged, 1.0f));
  
    float vSag = mSimState->stateInit.maxVoltageSag * powerFactor2 + // power dependency
                 (mSimState->stateInit.maxVoltageSag * chargeFactorInv * chargeFactorInv * powerFactor2); // charge state dependency

    // actual vbat - sag - fuluctuations
    mSimState->batteryState.batVoltageSag = mSimState->batteryState.batVoltage - vSag - std::abs(randf(mSimState->rngState) * 0.01f);
    mSimState->batteryState.batVoltageSag = clamp(mSimState->batteryState.batVoltageSag, 0.0f, 100.0f);
    
    float currentSum = 0.0f;
    for(int i = 0; i < 4; i++){
      currentSum += std::abs(mSimState->motorsState[i].current);
    }

    double currentmAs = currentSum / 3.6f;

    // minimum consumption + random fluctuation clamped to max 1mA/s to account for running electronics
    const double mAMin = std::min(0.2, (0.5 + randf(mSimState->rngState) * 0.25) / std::max(mSimState->batteryState.batVoltageSag, 0.01f));
    currentmAs = std::max(currentmAs, mAMin );


    // 1W = 1V * 1A
    // P = I * V

    // milliAmpSeconds * 3600 / 1000
    mSimState->batteryState.amperage = currentmAs * 3.6;
    mSimState->batteryState.batCapacity -= currentmAs * dt;
    mSimState->batteryState.mAhDrawn = mSimState->stateInit.quadBatCapacityCharged - mSimState->batteryState.batCapacity;

    // negative cappa allows to drop voltage below 3.5V
    //batCapacity = std::max(batCapacity, 0.1f);
  }

  // calculates next phase 
  float Physics::shiftedPhase(const double dt, float hz, float phaseStart){
    constexpr float pi2 = M_PI * 2.0f;
    float phaseShift = (pi2 * dt * hz);
    float phaseUpdated = phaseStart + phaseShift;
    if(std::abs(phaseUpdated) > pi2){ // keep number low. is used for sin/cos anyways
      phaseUpdated = phaseUpdated - (pi2 * static_cast<int>(phaseUpdated / pi2));
    }
    return phaseUpdated;
  }

  mat3 Physics::motorNoise(const double dt, MotorState& motor){
    // update phase and harmonics
    motor.phase          = shiftedPhase(dt, rpmToHz(motor.rpm)       , motor.phase);
    motor.phaseHarmonic1 = shiftedPhase(dt, rpmToHz(motor.rpm) * 2.0f, motor.phaseHarmonic1);
    motor.phaseHarmonic2 = shiftedPhase(dt, rpmToHz(motor.rpm) * 3.0f, motor.phaseHarmonic2);

    motor.phaseSlow      = shiftedPhase(dt, rpmToHz(motor.rpm) * 0.01f, motor.phaseSlow);

    float sinPhase = sinf(motor.phase);
    float sinPhaseH1 = sinf(motor.phaseHarmonic1);
    float sinPhaseH2 = sinf(motor.phaseHarmonic2);

    float cosPhase = cosf(motor.phase);
    float cosPhaseH1 = cosf(motor.phaseHarmonic1);
    float cosPhaseH2 = cosf(motor.phaseHarmonic2);

    return {vec3{sinPhase, sinPhaseH1, sinPhaseH2}, 
            vec3{cosPhase, cosPhaseH1, cosPhaseH2},
            vec3{sinPhase + cosPhase, sinPhaseH1 + cosPhaseH1, sinPhaseH2 + cosPhaseH2}};
  }

  void Physics::updateGyroNoise(const StateInput& state, vec3& angularNoise){
    // white noise
    float whiteNoiseX = randf(mSimState->rngState) * state.gyroBaseNoiseAmp;
    float whiteNoiseY = randf(mSimState->rngState) * state.gyroBaseNoiseAmp;
    float whiteNoiseZ = randf(mSimState->rngState) * state.gyroBaseNoiseAmp;

    angularNoise[0] = whiteNoiseX;
    angularNoise[1] = whiteNoiseY;
    angularNoise[2] = whiteNoiseZ;
  }

  void Physics::updateMotorNoise(const double dt, const StateInput& state, vec3& angularNoise){
    float maxV = mSimState->stateInit.quadBatCellCount * 4.2;

    // per motor 0 - 3
    vec4 motorKV = toVec4(mSimState->stateInit.motorKV);
    vec4 maxRpm = maximum((motorKV * maxV), 0.1f);

    vec4 motorRpm = {
      mSimState->motorsState[0].rpm,
      mSimState->motorsState[1].rpm,
      mSimState->motorsState[2].rpm,
      mSimState->motorsState[3].rpm
    };

    vec4 rpmFactor    = maximum(motorRpm, 0.0f) / maxRpm;
    vec4 rpmFactor2   = rpmFactor * rpmFactor;
    // effective damage (client input + impact-accumulated) drives vibration
    vec4 dmgFactor    = toVec4(mSimState->damage.eff) + 0.05f;
    vec4 rpmDmgFactor = dmgFactor * rpmFactor2;

    // only call once per dt, adapts motor phase!
    std::array<mat3, 4> mNoise = {
      motorNoise(dt, mSimState->motorsState[0]),
      motorNoise(dt, mSimState->motorsState[1]),
      motorNoise(dt, mSimState->motorsState[2]),
      motorNoise(dt, mSimState->motorsState[3])
    };

    vec3 noise{0.0f, 0.0f, 0.0f};
    for(int i = 0; i < 4; i++){
      noise[0] += 
        // noise
        mNoise[i][0][0] * state.motorImbalance[i].x * rpmDmgFactor[i] +
        // harmonic 1
        mNoise[i][0][1] * state.motorImbalance[i].x * rpmDmgFactor[i] * mSimState->stateInit.propHarmonic1Amp +
        // harmonic 2
        mNoise[i][0][2] * state.motorImbalance[i].x * rpmDmgFactor[i] * mSimState->stateInit.propHarmonic2Amp;

      noise[1] +=
        // motor noise
        mNoise[i][1][0] * state.motorImbalance[i].y * rpmDmgFactor[i] +
        // harmonic 1
        mNoise[i][1][1] * state.motorImbalance[i].y * rpmDmgFactor[i] * mSimState->stateInit.propHarmonic1Amp +
        // harmonic 2
        mNoise[i][1][2] * state.motorImbalance[i].y * rpmDmgFactor[i] * mSimState->stateInit.propHarmonic2Amp;

      noise[2] += ( 
        // motor noise
        mNoise[i][2][0] * state.motorImbalance[i].z * rpmDmgFactor[i] +
        //harmonic 1
        mNoise[i][2][1] * state.motorImbalance[i].z * rpmDmgFactor[i] * mSimState->stateInit.propHarmonic1Amp +
        //harmonic 2
        mNoise[i][2][2] * state.motorImbalance[i].z * rpmDmgFactor[i] * mSimState->stateInit.propHarmonic2Amp) * 0.5f;
    }

    // frame noise 
    mSimState->frameHarmonicPhase1 = shiftedPhase(dt, state.frameHarmonic1Freq + randf(mSimState->rngState) * 70.0f, mSimState->frameHarmonicPhase1);
    mSimState->frameHarmonicPhase2 = shiftedPhase(dt, state.frameHarmonic2Freq + randf(mSimState->rngState) * 60.0f, mSimState->frameHarmonicPhase2);

    vec4 rpmFactorHDec  = minimum(maximum(motorRpm, 0.0f) / (maxRpm * 0.15f), 1.0f);
    float rpmFactorH = sum(rpmFactorHDec) * 0.25f;

    vec4 rpmFactorH1Inc  = minimum(maximum(motorRpm, 0.0f) / (maxRpm * 0.43f), 1.0f);
    float rpmFactorH1Inv = sum(1.0f - rpmFactorH1Inc) * 0.25f;

    vec4 rpmFactorH2Inc  = minimum(maximum(motorRpm, 0.0f) / (maxRpm * 0.3f), 1.0f);
    float rpmFactorH2Inv =  sum(1.0f - rpmFactorH2Inc) * 0.25f;

    noise[0] += //frame harmonic 1
      ( state.motorImbalance[0].x * dmgFactor[0] +
        state.motorImbalance[1].x * dmgFactor[1] +
        state.motorImbalance[2].x * dmgFactor[2] +
        state.motorImbalance[3].x * dmgFactor[3]
      ) * 0.25f * state.frameHarmonic1Amp * sinf(mSimState->frameHarmonicPhase1) * rpmFactorH1Inv * rpmFactorH;
    noise[0] +=//frame harmonic 2
      ( state.motorImbalance[0].x * dmgFactor[0] +
        state.motorImbalance[1].x * dmgFactor[1] +
        state.motorImbalance[2].x * dmgFactor[2] +
        state.motorImbalance[3].x * dmgFactor[3]
      ) * 0.25f * state.frameHarmonic2Amp * sinf(mSimState->frameHarmonicPhase2)  * rpmFactorH2Inv * rpmFactorH;

    noise[1] += //frame harmonic 1
      ( state.motorImbalance[0].y * dmgFactor[0] +
        state.motorImbalance[1].y * dmgFactor[1] +
        state.motorImbalance[2].y * dmgFactor[2] +
        state.motorImbalance[3].y * dmgFactor[3]
      ) * 0.25f * state.frameHarmonic1Amp * cosf(mSimState->frameHarmonicPhase1) * rpmFactorH1Inv * rpmFactorH;
    noise[1] += //frame harmonic 2
      ( state.motorImbalance[0].y * dmgFactor[0] +
        state.motorImbalance[1].y * dmgFactor[1] +
        state.motorImbalance[2].y * dmgFactor[2] +
        state.motorImbalance[3].y * dmgFactor[3]
      ) * 0.25f * state.frameHarmonic2Amp * cosf(mSimState->frameHarmonicPhase2)  * rpmFactorH2Inv * rpmFactorH;

    noise[2] += //frame harmonic 1
      ( state.motorImbalance[0].z * dmgFactor[0] +
        state.motorImbalance[1].z * dmgFactor[1] +
        state.motorImbalance[2].z * dmgFactor[2] +
        state.motorImbalance[3].z * dmgFactor[3]
      ) * 0.25f * state.frameHarmonic1Amp * sinf(mSimState->frameHarmonicPhase1) * cosf(mSimState->frameHarmonicPhase1) * rpmFactorH1Inv * rpmFactorH;
    noise[2] += //frame harmonic 2
      ( state.motorImbalance[0].z * dmgFactor[0] +
        state.motorImbalance[1].z * dmgFactor[1] +
        state.motorImbalance[2].z * dmgFactor[2] +
        state.motorImbalance[3].z * dmgFactor[3]
      ) * 0.25f * state.frameHarmonic2Amp * sinf(mSimState->frameHarmonicPhase2) * cosf(mSimState->frameHarmonicPhase2) * rpmFactorH2Inv * rpmFactorH;

    //BF_DEBUG_SET(bf::DEBUG_SIM, 3, (1.0f + noiseX) * 1000.0f);
    //BF_DEBUG_SET(bf::DEBUG_SIM, 2, m1Hz);

    angularNoise = noise;
  }

  float Physics::calculateMotors(double dt,
                              StateInput& state,
                              std::array<MotorState, 4>& motors) 
  {
    // propwash: flipped vs SimITL ({1,-1,-1,1}) — with Betaflight 4.5.2's
    // default props-in quad-X mixer (yaw signs {-1,1,1,-1} for RR,FR,RL,FL)
    // the SimITL signs produced yaw positive feedback here (diagonal motor
    // pair saturates and the quad climbs at min throttle).
    const float motor_dir[4] = {-1.0, 1.0, 1.0, -1.0};

    float resPropTorque = 0;

    mat3 rotation;
    copy(rotation, state.rotation);
    const auto up = get_axis(rotation, 1);

    vec3 linVel;
    copy(linVel, state.linearVelocity);
    // aerodynamics act on the velocity relative to the AIR, not the ground
    const vec3 airVel = linVel - mSimState->wind;
    const auto vel = std::max(0.0f, dot(airVel, up));

    const auto ambientTemp = mSimState->stateInit.ambientTemp;
    // prevent div by 0 
    const float propWashAngleOfAttack = std::min(mSimState->stateInit.propWashAngleOfAttack, 0.99f);
    const float minPropWashSpeed = mSimState->stateInit.minPropWashSpeed;
    const float maxPropWashSpeed = std::max(mSimState->stateInit.maxPropWashSpeed, 0.001f); 
    const float propWashFactor   = mSimState->stateInit.propWashFactor;

    const float speed = std::abs(length(airVel)); // airspeed, m/s
    float speedFactor = std::min(speed / maxPropWashSpeed, 1.0f);

    for (int i = 0; i < 4; i++) {

      // effective damage: max(client-scripted input, impact-accumulated)
      const float propDamage = mSimState->damage.eff[i];

      // 1.0 - effect
      float propHealthFactor = 1.0f - propDamage;
      // 1.0 + effect: increasing torque for damaged prop
      float propHealthTorqueFactor = 1.0f + propDamage;

      // 1.0 + effect: increasing thrust close to ground
      float groundEffect = 1.0f + ((state.groundEffect[i] * state.groundEffect[i]) * 0.7f);
      
      // clamp speed so it has no propwash effect at 0
      // positive value depending on how much thrust is given against actual movement direction of quad
      float reverseThrust = (speed > minPropWashSpeed) ?
        std::max(0.0f, dot(normalize(airVel), normalize(motors[i].thrust * up) * -1.0f)) : 0.0f;
      // keep between 0.0 and 1.0, takes x % that point the most against movement direction
      
      reverseThrust = std::max(0.0f, (reverseThrust - propWashAngleOfAttack) / (1.0f - propWashAngleOfAttack));
      reverseThrust = reverseThrust * reverseThrust;

      float motorPhaseCompressed = static_cast<float>(static_cast<int>(motors[i].phaseSlow * 4.0f)) /  4.0f;

      float propWashNoise = motors[i].propWashLowPassFilter.update( 
        std::min(1.0f, std::max(0.0f, std::abs(SimplexNoise::noise(motorPhaseCompressed)))), 
        dt, 
        45.0f
      );

      // 1.0 - effect
      float propwashEffect = std::max(std::min(1.0f - (speedFactor * propWashNoise * reverseThrust) * propWashFactor, 1.0f), 0.0f);

      // 1.0 - effect    reusing prop wash noise to reduce thrust if motor/prop is damaged
      float motorDamageEffect = 1.0f - (std::max(0.0f, 0.5f * (SimplexNoise::noise(motors[i].phase * speed) + 1.0f)) * propDamage * propWashNoise);

      // 1.0 - effect   reusing prop wash noise to reduce thrust if motor is damaged
      float propDamageEffect = 1.0f - (propDamage * propWashNoise);

      auto rpm = motors[i].rpm;
      const auto kV = mSimState->stateInit.motorKV[i];
      const auto R  = mSimState->stateInit.motorR[i];
      const auto I0 = mSimState->stateInit.motorI0[i];
      const auto Rth = mSimState->stateInit.motorRth;
      const auto Cth = mSimState->stateInit.motorCth;

      //prevent division by 0
      float vbat = std::max(1.0f, mSimState->batteryState.batVoltageSag);

      float armed = mSimState->armed ? 0.0f : 1.0f;

      const auto volts = motors[i].pwmLowPassFilter.update(motors[i].pwm, dt, 100.0f) * vbat;
      const auto mTorque = motorTorque(volts, rpm, kV, R, I0) * 0.833f * motorDamageEffect;
      auto current       = motorCurrent(mTorque, kV);
      const auto pTorque = propTorque(rpm, vel) * propHealthTorqueFactor;
      const auto netTorque = mTorque - pTorque;

      const auto domega = netTorque / std::max(mSimState->stateInit.propInertia, 0.00000001f);
      const auto drpm = (domega * dt) * 60.0f / (2.0f * float(M_PI));

      const auto maxdrpm = fabsf(volts * kV - rpm);
      rpm += clamp(drpm, -maxdrpm, maxdrpm);

      if((motors[i].status & MotorStatus::MotorBurnedOut) != 0){
        rpm = 0.0f;
        current = 0.0f;
      }

      float currentAbs = std::abs(current);
      float thrust = propThrust(rpm, vel) * propHealthFactor * groundEffect * propwashEffect * propDamageEffect;
      float powerDraw = currentAbs * vbat;

      float cooling = (1.0f - std::exp(-speed * 0.2f)) * 100.0f; // 100 watts max cooling by airspeed
      constexpr float maxSpeedPropCooling = 20.0f; //72 km/h and cooling by rotation of props has no effect
      cooling += (std::min(maxSpeedPropCooling, speed) / maxSpeedPropCooling) * thrust * 4.0f;

      // motor pwm is already read by BF::update call
      //motors[i].pwm = bf::motorsPwm[i] / 1000.0f;

      //heating
      motors[i].temp += ( std::max(0.0f,  powerDraw - cooling) - (motors[i].temp - ambientTemp) / Rth) / Cth  * dt;

      motors[i].current = current;
      motors[i].pTorque = pTorque;
      motors[i].mTorque = mTorque;
      motors[i].thrust = thrust;
      motors[i].rpm = rpm;
      resPropTorque += motor_dir[i] * mTorque;

      if(motors[i].temp > mSimState->stateInit.motorMaxT){
        motors[i].status = motors[i].status | MotorStatus::MotorBurnedOut;
      }

      if(i == 3){
        BF::setDebugValue(E_DEBUG_SIM, 0, reverseThrust    * 1000);
        BF::setDebugValue(E_DEBUG_SIM, 1, propDamageEffect * 1000);
        // BF::setDebugValue(E_DEBUG_SIM, 2, mTorque          * 1000);
        BF::setDebugValue(E_DEBUG_SIM, 3, propwashEffect   * 1000);
      }
      
    }

    BF::setDebugValue(E_DEBUG_SIM, 4, motors[0].thrust * 1000);
    BF::setDebugValue(E_DEBUG_SIM, 5, motors[1].thrust * 1000);
    BF::setDebugValue(E_DEBUG_SIM, 6, motors[2].thrust * 1000);
    BF::setDebugValue(E_DEBUG_SIM, 7, motors[3].thrust * 1000);

    return resPropTorque;
  }

  vec3 Physics::calculatePhysics(
    double dt,
    StateInput& state,
    const std::array<MotorState, 4>& motors,
    float motorsTorque
  ) {
    vec3 acceleration;

    auto gravity_force = vec3{0, -9.81f * mSimState->stateInit.quadMass, 0};

    // force sum:
    vec3 total_force = gravity_force;

    // drag — on the velocity relative to the air, so wind both drifts the
    // quad and changes its effective drag
    vec3 linearVelocity;
    copy(linearVelocity, state.linearVelocity);
    const vec3 airVelocity = linearVelocity - mSimState->wind;

    float vel2 = length2(airVelocity);
    auto dir = normalize(airVelocity);

    mat3 rotation;
    copy(rotation, state.rotation);
    auto local_dir = xform_inv(rotation, dir);

    vec3 frameDragArea;
    copy(frameDragArea, mSimState->stateInit.frameDragArea);
    float areaLinear = dot(frameDragArea, abs(local_dir));
    float areaAngular = dot(frameDragArea, local_dir);

    vec3 dragDir = dir * 0.5f * AIR_RHO * vel2 * mSimState->stateInit.frameDragConstant;
    vec3 dragLinear = dragDir * areaLinear;
    vec3 dragAngular = dragDir * areaAngular;
    total_force = total_force - dragLinear;

    // motors:
    for (auto i = 0u; i < 4; i++) {
      total_force = total_force + xform(rotation, vec3{0, motors[i].thrust, 0});
    }

    // contacts: penalty forces at the manifold points. Added to the same
    // accumulator as thrust so they flow into `acceleration` — which is what
    // the virtual accelerometer reads: the firmware feels touchdown, ground
    // support and crashes exactly like real hardware.
    vec3 contactMoment {0.0f, 0.0f, 0.0f};
    total_force = total_force + contactForces(dt, state, contactMoment);

    acceleration = total_force / std::max(mSimState->stateInit.quadMass, 0.001f);

    linearVelocity = linearVelocity + acceleration * dt;
    assert(std::isfinite(length(linearVelocity)));
    copy(state.linearVelocity, linearVelocity);
    
    // moment sum around origin:
    vec3 total_moment = get_axis(rotation, 1) * motorsTorque * 4.0f;

    // drag induced momentum
    dragAngular = xform_inv(rotation, dragAngular) * 0.001f;
    dragAngular = clamp(dragAngular, -0.9f, 0.9f);

    total_moment = total_moment + get_axis(rotation, 0) * dragAngular[1];
    total_moment = total_moment + get_axis(rotation, 1) * dragAngular[0];
    total_moment = total_moment + get_axis(rotation, 2) * dragAngular[2];

    for (auto i = 0u; i < 4; i++) {
      auto force = xform(rotation, {0, motors[i].thrust, 0});
      auto rad = xform(rotation, motors[i].position);
      total_moment = total_moment + cross(rad, force);
    }

    total_moment = total_moment + contactMoment;

    // prop-strike yaw kick: a striking prop grabs the surface briefly
    if (mSimState->damage.strikeTicks > 0) {
      total_moment = total_moment + get_axis(rotation, 1) *
          (mSimState->damageCfg.strikeTorque * mSimState->damage.strikeYawSign);
    }

    vec3 inv_inertia;
    copy(inv_inertia, mSimState->stateInit.quadInvInertia);
    mat3 inv_tensor = {vec3{inv_inertia[0], 0, 0},
                      vec3{0, inv_inertia[1], 0},
                      vec3{0, 0, inv_inertia[2]}};
    inv_tensor = rotation * inv_tensor * transpose(rotation);
    vec3 angularAcc = xform(inv_tensor, total_moment);
    assert(std::isfinite(angularAcc[0]) && std::isfinite(angularAcc[1]) &&
      std::isfinite(angularAcc[2]));

    vec3 angularVelocity;
    copy(angularVelocity, state.angularVelocity);       // test drag rotation
    angularVelocity = angularVelocity + angularAcc * dt;

    angularVelocity = clamp(angularVelocity, -100.0f, 100.0f);

    copy(state.angularVelocity, angularVelocity);

    updateRotation(dt, state);
    advanceContactDepths(dt, state);
    return acceleration;
  }

  vec3 Physics::contactForces(double dt, const StateInput& state, vec3& momentOut){
    vec3 totalForce {0.0f, 0.0f, 0.0f};
    if (mSimState->contactCount == 0) {
      return totalForce;
    }

    mat3 rotation;
    copy(rotation, state.rotation);
    vec3 linVel;
    copy(linVel, state.linearVelocity);
    vec3 angVel;
    copy(angVel, state.angularVelocity);

    for (auto& c : mSimState->contacts) {
      if (!c.active) continue;
      const ContactMaterial& mat = contactMaterial(c.surface);

      c.rWorld = xform(rotation, c.pointBody);
      const vec3 vPoint = linVel + cross(angVel, c.rWorld);
      const float vN = dot(vPoint, c.normalWorld);

      // spring-damper along the normal; max(0,..) means no adhesion
      const float fN = clamp(mat.k * c.depth - mat.c * vN, 0.0f, CONTACT_FMAX);

      // Coulomb friction with a viscous cap for stability near rest
      const vec3 vT = vPoint - c.normalWorld * vN;
      const float vTLen = length(vT);
      vec3 fT {0.0f, 0.0f, 0.0f};
      if (vTLen > 1e-6f) {
        fT = (vT / vTLen) * -std::min(mat.mu * fN, CONTACT_KT * vTLen);
      }

      const vec3 f = c.normalWorld * fN + fT;
      c.force = f;
      totalForce = totalForce + f;
      momentOut = momentOut + cross(c.rWorld, f);

      const float fLen = length(f);
      if (fLen > mSimState->contactPeakForceN) {
        mSimState->contactPeakForceN = fLen;
      }
      mSimState->contactImpulseNs += fLen * static_cast<float>(dt);
    }
    return totalForce;
  }

  void Physics::advanceContactDepths(double dt, const StateInput& state){
    if (mSimState->contactCount == 0) {
      return;
    }
    vec3 linVel;
    copy(linVel, state.linearVelocity);
    vec3 angVel;
    copy(angVel, state.angularVelocity);

    for (auto& c : mSimState->contacts) {
      if (!c.active) continue;
      // rWorld cached by contactForces this sub-tick; the surface is static,
      // so the point sinks/rises with its own normal velocity
      const vec3 vPoint = linVel + cross(angVel, c.rWorld);
      const float vN = dot(vPoint, c.normalWorld);
      c.depth += static_cast<float>(-vN * dt);
      /* A separated point stays ARMED at depth 0 until the next manifold:
       * with d = 0 the spring term vanishes and Fn = max(0, -c*vn) is a
       * one-sided damper on re-approach. Expiring immediately let a rocking
       * quad re-impact undamped between client frames, which pumped a
       * standing wobble that never settled. */
      c.depth = clamp(c.depth, 0.0f, CONTACT_DMAX);
    }
  }

  void Physics::updateCommands(CommandType commands){
    switch (commands)
    {
    case CommandType::Repair:
      repair();
      break;
    case CommandType::Reset:
      reset();
      break;
    default:
      break;
    }
  }

  void Physics::repair(){
    mSimState->batteryState.batCapacity = mSimState->stateInit.quadBatCapacityCharged;
    for(int i = 0; i < 4; i++){
      mSimState->motorsState[i].temp = mSimState->stateInit.ambientTemp;
      mSimState->motorsState[i].status = MotorStatus::MotorNone;
    }
    // new props on, crash latch cleared
    mSimState->damage = DamageState{};
  }

  /* Clears the integrator state that initState() does not touch. Without this,
   * a "reset" run kept motor phases, filter histories and harmonic phases from
   * the previous flight and so diverged from a freshly started process — which
   * is the whole point of a reset. */
  void Physics::reset(){
    mSimState->acceleration = {0.0f, 0.0f, 0.0f};
    mSimState->stateInput.linearVelocity = {0.0f, 0.0f, 0.0f};
    mSimState->stateInput.angularVelocity = {0.0f, 0.0f, 0.0f};

    for (auto& c : mSimState->contacts) {
      c = ActiveContact{};
    }
    mSimState->contactCount = 0;
    mSimState->contactPeakForceN = 0.0f;
    mSimState->contactImpulseNs = 0.0f;
    mSimState->damage = DamageState{};
    mSimState->bfCrashRecoveryActive = false;
    mSimState->bfFlipOverActive = false;

    mSimState->gyro = {0.0f, 0.0f, 0.0f};
    mSimState->acc = {0.0f, 0.0f, 0.0f};
    mSimState->gyroNoise = {0.0f, 0.0f, 0.0f};
    mSimState->motorNoise = {0.0f, 0.0f, 0.0f};
    mSimState->combinedNoise = {0.0f, 0.0f, 0.0f};
    mSimState->frameHarmonicPhase1 = 0.0f;
    mSimState->frameHarmonicPhase2 = 0.0f;

    for (auto i = 0u; i < 3; i++) {
      mSimState->gyroLowPassFilter[i] = LowPassFilter{};
    }
    for (auto i = 0u; i < 4; i++) {
      auto& m = mSimState->motorsState[i];
      m.pwm = 0.0f;
      m.rpm = 0.0f;
      m.current = 0.0f;
      m.thrust = 0.0f;
      m.phase = 0.0f;
      m.phaseHarmonic1 = 0.0f;
      m.phaseHarmonic2 = 0.0f;
      m.phaseSlow = 0.0f;
      m.pwmLowPassFilter = LowPassFilter{};
      m.propWashLowPassFilter = LowPassFilter{};
    }
  }
}