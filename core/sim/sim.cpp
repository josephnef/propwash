#include "sim.h"

#include <cstdint>
#include <cstdio>

namespace SimITL{

  // 20 kHz firmware/physics tick. Must be HIGHER than the gyro/PID rate
  // (8 kHz): the scheduler only runs non-realtime tasks (RX, MSP, ...) on
  // calls that land BETWEEN gyro boundaries (schedLoopRemainingCycles >
  // CHECK_GUARD_MARGIN). Ticking exactly at the gyro rate starves them.
  // SimITL and KwadSimServer both tick at 20 kHz for the same reason.
  const int64_t FREQUENCY = 20e3;
  const int64_t DELTA = 1e6 / FREQUENCY;

  Sim& Sim::getInstance() {
    static Sim simulator;
    return simulator;
  }

  Sim::Sim()
  {
    mPhysics.setSimState(&mSimState);
  }

  Sim::~Sim() {
  }

  void Sim::init(const StateInit& stateInit) {
    mPhysics.initState(stateInit);

    //reset rc data to valid data...
    BF::resetRcData();

    printf("[pw] initializing betaflight\n");
    BF::setEepromFileName((const char *)stateInit.eepromName);
    BF::init();

    // The canonical firmware state: captured once, right here, before any
    // simulated time has passed. Every reset rewinds to this instead of
    // trusting BF::init() to clear the firmware's statics — it doesn't
    // (scheduler stats, IMU quaternion, PID state, gyro calibration, RX
    // latches, OSD timers all survive an in-process re-init, which made
    // every reset start from a slightly different state).
    BF::takeStateSnapshot();
  }

  void Sim::reinitPhysics(const StateInit& stateInit){
    mPhysics.initState(stateInit);
  }

  /* Everything a fresh process would have. BF::init() is the same entry point
   * pw_sitl.c's systemReset() calls, and systemInit() guards the dyad thread
   * spawn and zeroes pw_micros_passed, so re-running it in-process is safe —
   * this is the path the CLI `save` reboot already exercises. */
  void Sim::resetPhysicsOnly(const StateInit& stateInit){
    total_delta = 0;
    mPhysics.initState(stateInit);
    mPhysics.reset();
  }

  void Sim::reset(const StateInit& stateInit){
    total_delta = 0;              // drop any sub-tick residue
    // Rewind the firmware statics to the post-first-boot snapshot BEFORE
    // re-running init: init() alone leaves dozens of statics carrying the
    // previous session's history (calibration progress, scheduler stats,
    // IMU/PID state, RC latches), so how much time had passed before the
    // reset used to leak into the trajectory after it. The restore makes
    // reset ≡ fresh process by construction; init() then re-reads the
    // eeprom, so CLI-saved settings still survive.
    BF::restoreStateSnapshot();
    mPhysics.initState(stateInit);  // also reseeds the noise RNG
    mPhysics.reset();             // integrator/filter/phase state
    // resetRcData AFTER init, not before. Betaflight latches switch state
    // during init, so clearing the channels first and then initialising left
    // the previous flight's arm switch still flagged (ARM_SWITCH survived a
    // reset). Clearing afterwards gives the firmware neutral sticks to
    // evaluate against.
    BF::init();
    BF::resetRcData();
    BF::configureDefaultModes();
    BF::disableRunawayTakeoff();
  }

  void Sim::update(const StateInput& stateInput){
    if(!mPhysics.checkSimState()){
      return; // no SimState, no sim!
    }

    int64_t stateUpdateDelta = static_cast<int64_t>(stateInput.delta * 1000000.0);

    if(stateUpdateDelta > static_cast<int64_t>(100000)){
      stateUpdateDelta = static_cast<int64_t>(100000);
    }
    if(stateUpdateDelta < static_cast<int64_t>(0)){
      stateUpdateDelta = static_cast<int64_t>(1);
    }

    total_delta += stateUpdateDelta;

    //update rc data
    BF::setRcData(stateInput.rcData);

    mPhysics.updateState(stateInput);

    //rc data is updated independently
    simStep();
  }

  void Sim::updateMotors(const StateInput& stateInput, const float motor[4]){
    if(!mPhysics.checkSimState()){
      return; // no SimState, no sim!
    }

    int64_t stateUpdateDelta = static_cast<int64_t>(stateInput.delta * 1000000.0);
    if(stateUpdateDelta > static_cast<int64_t>(100000)){
      stateUpdateDelta = static_cast<int64_t>(100000);
    }
    if(stateUpdateDelta < static_cast<int64_t>(0)){
      stateUpdateDelta = static_cast<int64_t>(1);
    }

    total_delta += stateUpdateDelta;

    mPhysics.updateState(stateInput);

    // Same accumulator as simStep(), but the firmware is bypassed: the motor
    // commands come straight from the caller (a recorded blackbox log), so the
    // physics model is exercised in isolation from Betaflight's PID loop.
    while ((total_delta - DELTA) >= 0) {
      total_delta -= DELTA;
      const double dt = static_cast<double>(DELTA) / 1e6f;

      mPhysics.updateGyro(dt);
      for (int i = 0; i < 4; i++) {
        mSimState.motorsState[i].pwm = motor[i];
      }
      mPhysics.updatePhysics(dt);
    }

    mSimState.microsPassed += stateUpdateDelta;
  }

  const StateOutput& Sim::getStateUpdate() const{
    return mSimState.stateOutput;
  }

  void Sim::command(const CommandType cmd){
    mPhysics.updateCommands(cmd);
  };

  void Sim::simStep() {
    for (auto k = 0u; (total_delta - DELTA) >= 0; k++) {
      total_delta -= DELTA;
      const double dt = static_cast<double>(DELTA) / 1e6f;

      mPhysics.updateGyro(dt);

      // updates betaflight data and schedules bf update
      BF::update(DELTA, mSimState);

      mPhysics.updatePhysics(dt);
    }
  }

}
