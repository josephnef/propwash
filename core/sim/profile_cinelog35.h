/**
 * Physical profile: GEPRC CineLog35 V3 (3.5" ducted cinewhoop).
 *
 * Starting values derived from SimITL's 5" reference profile, scaled to the
 * CineLog35: GEPRC SPEED 2105.5 2650KV motors, 3.5" tri-blade props,
 * 4S 660 mAh pack, AUW ~330 g. These are M1-grade estimates — M5 (blackbox
 * system identification against the real quad) is what makes them honest.
 */

#ifndef PW_PROFILE_CINELOG35_H
#define PW_PROFILE_CINELOG35_H

#include <cstring>
#include <algorithm>
#include "sim_packets.h"

inline void profileCineLog35(StateInit& s, const char* eepromPath)
{
  // Motor: GEPRC SPEED 2105.5 2650KV
  for (int i = 0; i < 4; i++) {
    s.motorKV[i] = 2650.0f;
    s.motorR[i]  = 0.09f;
    s.motorI0[i] = 0.007f;
  }
  s.motorRth  = 0.3f;
  s.motorCth  = 20.0f;
  s.motorMaxT = 128.0f;

  // Propeller: 3.5" tri-blade (ducted).
  // propThrustFactor.z = static max thrust per prop in N at propMaxRpm.
  // CineLog35: ~1 kg total max thrust (TWR ~3:1 at 330 g AUW) -> 2.8 N/prop.
  // Loaded max rpm ~0.75 * kV * 16.8 V ~= 33000.
  s.propBladeCount   = 3;
  s.propMaxRpm       = 33000.0f;
  s.propAFactor      = 1.6e-9f;
  s.propTorqueFactor = 0.0070f;
  s.propInertia      = 1.6e-7f;
  s.propThrustFactor = { -0.000004f, -0.05f, 2.8f };
  s.propHarmonic1Amp = 0.1f;
  s.propHarmonic2Amp = 0.3f;

  // Frame: ducted cinewhoop — more drag area than an open 5"
  s.frameDragArea     = { 0.0110f, 0.0135f, 0.0110f };
  s.frameDragConstant = 1.35f;
  s.quadMass          = 0.33f;   // AUW with 4S 660 mAh
  s.quadInvInertia    = { 900.0f, 1100.0f, 900.0f };

  // Motor positions: 153 mm wheelbase -> ±54 mm x/z
  s.quadMotorPos[0] = {  0.054f, 0.005f, -0.054f };
  s.quadMotorPos[1] = {  0.054f, 0.005f,  0.054f };
  s.quadMotorPos[2] = { -0.054f, 0.005f, -0.054f };
  s.quadMotorPos[3] = { -0.054f, 0.005f,  0.054f };

  // Battery: 4S 660 mAh
  s.maxVoltageSag          = 1.6f;
  s.quadBatCellCount       = 4;
  s.quadBatCapacityCharged = 660.0f;
  s.quadBatCapacity        = 660.0f;

  // Reversed-spin (crashflip) thrust penalty — ducted props suffer more
  s.propReverseEfficiency = 0.7f;

  // Prop wash — ducts wash earlier and harder when descending
  s.minPropWashSpeed      = 0.8f;
  s.maxPropWashSpeed      = 10.0f;
  s.propWashAngleOfAttack = 0.1f;
  s.propWashFactor        = 1.2f;

  s.ambientTemp = 25.0f;

  std::fill(s.eepromName, s.eepromName + sizeof(s.eepromName), 0);
  std::strncpy(reinterpret_cast<char*>(s.eepromName), eepromPath, sizeof(s.eepromName) - 1);
}

inline void inputDefaults(StateInput& s)
{
  s.delta = 0.016f;

  // RC channels -1..1 (mapped to 1000-2000): AETR, sticks safe
  s.rcData[0] =  0.0f; // roll
  s.rcData[1] =  0.0f; // pitch
  s.rcData[2] = -1.0f; // throttle low
  s.rcData[3] =  0.0f; // yaw
  s.rcData[4] = -1.0f; // ch5 ARM off
  s.rcData[5] = -1.0f; // ch6 ANGLE off
  s.rcData[6] = -1.0f;
  s.rcData[7] = -1.0f;

  // Identity rotation
  s.rotation[0] = { 1.0f, 0.0f, 0.0f };
  s.rotation[1] = { 0.0f, 1.0f, 0.0f };
  s.rotation[2] = { 0.0f, 0.0f, 1.0f };

  // Motor imbalance / noise — SimITL reference values
  for (int i = 0; i < 4; i++) {
    s.motorImbalance[i] = { 13.0f, 7.0f, 5.0f };
  }
  s.gyroBaseNoiseAmp  = 0.000287f;
  s.gyrobaseNoiseFreq = 228.0f;
  s.frameHarmonic1Amp  = 0.02242f;
  s.frameHarmonic1Freq = 275.0f;
  s.frameHarmonic2Amp  = 0.01f;
  s.frameHarmonic2Freq = 326.66f;

  s.vbat = 16.8f; // charged 4S
}

#endif // PW_PROFILE_CINELOG35_H
