/**
 * Betaflight glue — ported from SimITL's bf.cpp (GPL-3.0), adapted to
 * Betaflight 4.5.2 and the propwash target layer (pw_sitl.c).
 *
 * Differences from SimITL:
 *  - no websocket serial (Configurator connects via dyad TCP, UART1=5761)
 *  - no BF-side battery/GPS/OSD injection yet (battery lives in physics;
 *    the CineLog35 has no GPS; OSD arrives with M3)
 */
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <array>

#include "bf.h"
#include "bfbridge.h"
#include "static_snapshot.h"

namespace SimITL{

  #ifndef M_PI
  #define M_PI 3.14159265358979
  #endif

  const static auto GYRO_SCALE = 16.4f;
  const static auto RAD2DEG = (180.0f / float(M_PI));
  const static auto ACC_SCALE = (256 / 9.80665f);

  namespace BF {

    void resetRcData(){
      //reset rc data to valid data...
      for(int i = 0; i < SIMULATOR_MAX_RC_CHANNELS; i++){
        rcDataCache[i] = 1000U;
      }
    }

    void setRcData(const float (&data)[8])
    {
      uint32_t timeUs = pw_micros_passed & 0xFFFFFFFF;

      for (int i = 0; i < 8; i++)
      {
        rcDataCache[i] = uint16_t(1500 + data[i] * 500);
      }
      rcDataReceptionTimeUs = timeUs;
      // hack to trick bf into using sim data...
      BF::rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS;
      BF::rxRuntimeState.rcReadRawFn = BF::rxRcReadData;
      BF::rxRuntimeState.rcFrameStatusFn = BF::rxRcFrameStatus;
      BF::rxRuntimeState.rxProvider = BF::RX_PROVIDER_UDP;
      BF::rxRuntimeState.lastRcFrameTimeUs = timeUs;
    }

    void setEepromFileName(const char* filename){
      printf("[pw] eeprom path: %s\n", filename);
      BF::pw_set_eeprom_path(filename);
    }

    void updateGyroAcc(const SimState& simState){
      int16_t x, y, z;

      x = int16_t(BF::constrain(int(-simState.acc[2] * ACC_SCALE), -32767, 32767));
      y = int16_t(BF::constrain(int(simState.acc[0] * ACC_SCALE), -32767, 32767));
      z = int16_t(BF::constrain(int(simState.acc[1] * ACC_SCALE), -32767, 32767));
      BF::virtualAccSet(BF::virtualAccDev, x, y, z);

      x = int16_t(BF::constrain(int(-simState.gyro[2] * GYRO_SCALE * RAD2DEG), -32767, 32767));
      y = int16_t(BF::constrain(int( simState.gyro[0] * GYRO_SCALE * RAD2DEG), -32767, 32767));
      z = int16_t(BF::constrain(int(-simState.gyro[1] * GYRO_SCALE * RAD2DEG), -32767, 32767));
      BF::virtualGyroSet(BF::virtualGyroDev, x, y, z);

      BF::imuSetAttitudeQuat(
          simState.rotation[3],
          -simState.rotation[2],
          -simState.rotation[0],
          simState.rotation[1]);
    }

    bool update(uint64_t dt, SimState& simState){
      bool schedulerExecuted = false;

      BF::pw_micros_passed += dt;

      updateGyroAcc(simState);

      if (BF::pw_sleep_timer > 0) {
        BF::pw_sleep_timer -= dt;
        BF::pw_sleep_timer = std::max(int64_t(0), BF::pw_sleep_timer);
      } else {
        BF::scheduler();
        schedulerExecuted = true;
      }

#ifdef PW_DEBUG_SCHED
      static uint64_t schedCount = 0, lastPrint = 0;
      if (schedulerExecuted) schedCount++;
      if (BF::pw_micros_passed - lastPrint >= 1000000) {
        lastPrint = BF::pw_micros_passed;
        printf("[dbg] t=%.1fs sched=%llu sleep=%lld rc0=%.0f rc4=%.0f rxsig=%d frameStatusCalls=%u provider=%d\n",
               BF::pw_micros_passed / 1e6, (unsigned long long)schedCount,
               (long long)BF::pw_sleep_timer,
               (double)BF::rcData[0], (double)BF::rcData[4],
               BF::rxIsReceivingSignal() ? 1 : 0,
               BF::pw_rc_frame_status_calls,
               (int)BF::rxRuntimeState.rxProvider);
        printf("[dbg]   sensorGyro=%d\n", BF::sensors(BF::SENSOR_GYRO) ? 1 : 0);
      }
#endif

      simState.armed = (BF::armingFlags & BF::ARMED) == BF::ARMED;
      simState.armingDisabledFlags = (int)BF::getArmingDisableFlags();
      simState.flightModeFlags = (int)BF::flightModeFlags;
      simState.beep = BF::isBeeperOn();
      // firmware crash state: contact forces reach the virtual gyro/accel,
      // so BF's own crash detection can fire — surface it to the client.
      // (Turtle can never activate on this target — DSHOT-gated — but the
      // accessor is wired so the flag lights up the day that changes.)
      simState.bfCrashRecoveryActive = BF::crashRecoveryModeActive();
      simState.bfFlipOverActive = BF::isFlipOverAfterCrashActive();
      simState.stateOutput.beep = simState.beep ? 1U : 0U;
      // capture the OSD character grid (fake max7456 displayport)
      for (int y = 0; y < 16; y++) {
        for (int x = 0; x < 30; x++) {
          simState.stateOutput.osd[y * 30 + x] = BF::osdScreen[y][x];
        }
      }

      simState.microsPassed = BF::pw_micros_passed;
      simState.motorsState[0].pwm = BF::pw_motors_pwm[0] / 1000.0f;
      simState.motorsState[1].pwm = BF::pw_motors_pwm[1] / 1000.0f;
      simState.motorsState[2].pwm = BF::pw_motors_pwm[2] / 1000.0f;
      simState.motorsState[3].pwm = BF::pw_motors_pwm[3] / 1000.0f;

      return schedulerExecuted;
    }

    void setDebugValue(uint8_t mode, uint8_t index, int16_t value){
      BF_DEBUG_SET(mode, index, value);
    }

    // Configure aux modes matching the CineLog35 switch plan (CONFIG.md):
    // ARM on ch5 (AUX1), ANGLE on ch6 (AUX2). Applied in RAM post-init —
    // equivalent to `aux` CLI entries without touching the eeprom.
    void configureDefaultModes(){
      BF::modeActivationCondition_t* arm = BF::modeActivationConditionsMutable(0);
      arm->modeId = BF::BOXARM;
      arm->auxChannelIndex = 0; // AUX1 = rc ch5
      arm->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
      arm->range.endStep   = CHANNEL_VALUE_TO_STEP(2100);

      BF::modeActivationCondition_t* angle = BF::modeActivationConditionsMutable(1);
      angle->modeId = BF::BOXANGLE;
      angle->auxChannelIndex = 1; // AUX2 = rc ch6
      angle->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
      angle->range.endStep   = CHANNEL_VALUE_TO_STEP(2100);

      BF::analyzeModeActivationConditions();
      printf("[pw] aux modes configured: ARM=AUX1, ANGLE=AUX2\n");
    }

    // Bench/test relaxation: runaway-takeoff prevention misfires against an
    // untuned physics model (a clean vertical punch has pidSum high and gyro
    // quiet — exactly its trigger signature). Re-enable once the physics is
    // system-identified against the real quad (M5).
    void disableRunawayTakeoff(){
      BF::pidConfigMutable()->runaway_takeoff_prevention = 0;
      printf("[pw] runaway takeoff prevention disabled (bench mode)\n");
    }

    void takeStateSnapshot(){
      if (SimITL::snapshotTaken()) return;
      // lifetime/concurrency hazards the restore must never touch: dyad's
      // connection state (realloc'd fd arrays, mutated by the pump thread)
      // and the dyad mutex (held during the restore itself)
      void* addr = nullptr;
      unsigned long size = 0;
      BF::pw_dyad_state_range(&addr, &size);
      SimITL::snapshotExclude(addr, (size_t)size);
      BF::pw_dyad_mutex_range(&addr, &size);
      SimITL::snapshotExclude(addr, (size_t)size);
      SimITL::snapshotTake();
    }

    bool restoreStateSnapshot(){
      if (!SimITL::snapshotTaken()) return false;
      // quiesce dyad: its pump thread wraps every dyad_update in this lock,
      // so holding it guarantees no dyad callback is mid-flight while the
      // firmware statics (including serial buffers) rewind
      BF::pw_dyad_lock();
      const bool ok = SimITL::snapshotRestore();
      BF::pw_dyad_unlock();
      return ok;
    }
  } // namespace BF
} // namespace SimITL
