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
  }

  void Sim::reinitPhysics(const StateInit& stateInit){
    mPhysics.initState(stateInit);
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
