#ifndef SIM_PACKETS_H
#define SIM_PACKETS_H

#include <cstdint>

enum CommandType : int32_t {
  None      = 0U,
  Stop      = 1U,
  Repair    = 2U,
  Reset     = 4U,
};

enum MotorStatus : int32_t {
  MotorNone = 0U,
  MotorBurnedOut = 1U,
  MotorDamaged = 2U,
};

struct Vec3F{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Vec4F{
  float x = 1.0f;
  float y = 0.0f;
  float z = 0.0f;
  float w = 1.0f;
};

struct GpsData{
  // 48.420876993590745, 10.711263681338096
  int32_t lat = 484208769;
  int32_t lon = 107112636;
  int32_t alt = 0;
};

// One hull/world contact, mirrors PwContact (client-detected, core-resolved).
// pointBody is in the quad's body frame; normalWorld is a unit vector in the
// world frame pointing from the surface toward the quad.
struct ContactPoint{
  Vec3F pointBody {};
  Vec3F normalWorld {};
  float depth = 0.0f;
  uint8_t surface = 0;
};

// initial quad settings.
struct StateInit{
  float motorKV[4]    = {}; // KV
  float motorR[4]     = {}; // resistance
  float motorI0[4]    = {}; // idle current
  float motorRth   = 0.0f; // thermal resistance (deg C per Watt)
  float motorCth   = 0.0f; // thermal heat capacity (joules per deg C)
  float motorMaxT  = 0.0f; // max temp in C when motor windings short out

  uint8_t propBladeCount = 0U;
  float propMaxRpm = 0.0f;
  float propAFactor = 0.0f;
  float propTorqueFactor = 0.0f;
  float propInertia = 0.0f;
  Vec3F propThrustFactor {};
  float propHarmonic1Amp = 0.0f;
  float propHarmonic2Amp = 0.0f;

  Vec3F frameDragArea {};
  float frameDragConstant = 0.0f;

  float quadMass = 0.0f;
  Vec3F quadInvInertia {};
  float maxVoltageSag = 0.0f;
  uint8_t quadBatCellCount = 1;
  //charged up capacity, can be lower or higher than quadBatCapacity
  float quadBatCapacityCharged  = 0.0f;
  //battery capacacity rating
  float quadBatCapacity = 0.0f;
  Vec3F quadMotorPos[4] {};

  // Wind: mean world-frame flow plus simplex-noise gusts. The gust field is
  // a pure function of sim time + seed (no RNG stream consumption), so calm
  // (all zero) builds are bit-identical to pre-wind builds and windy runs
  // are reproducible given the same flags.
  Vec3F windMean {};          // m/s, sim world frame
  float windGustAmp = 0.0f;   // m/s gust amplitude (0 = steady wind only)
  float windGustFreq = 0.5f;  // gust rate scale

  // Thrust efficiency of a reversed prop (crashflip): blades are cambered
  // and pitched for one direction, and ducts make reversed flow worse.
  float propReverseEfficiency = 0.7f;

  // prop wash starts at this speed in m/s
  float minPropWashSpeed = 1.0f;
  // prop wash reacheas peak at this speed in m/s
  float maxPropWashSpeed = 18.0f;
  // max prop wash angle of attack (0.0 = 180 degrees in reverse trust dir)
  // > 0.0 means propwash in trust dir is concentrated more towards reverse trust dir ( less than 180 deg)
  // < 0.0 prop was angle of attack is greater than 180 degrees so even angles in thrust dir have an effect
  float propWashAngleOfAttack = 0.5f;
  // multiplier for the final prop wash effect
  float propWashFactor = 1.0f;

  float ambientTemp = 0.0f;

  /* Noise RNG seed. 0 means "use the built-in default", so existing callers
   * that leave it zero still get a fixed, reproducible stream. Carried on the
   * wire as PwInit.seed, which was declared for exactly this and never read. */
  uint32_t seed = 0;

  GpsData gps {};

  //eeprom file name
  uint8_t eepromName[512] = {};
};

//runtime quad parameters - simulation input
struct StateInput{
  float delta = 0.0f;

  // Normalised RC channel values in the range -1.0 to 1.0.
  float rcData[8] {};

  Vec3F position {};
  Vec3F rotation[3] {};

  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  // length is amplitude per axis
  Vec3F motorImbalance[4] {};

  float gyroBaseNoiseAmp = 0.0f;
  float gyrobaseNoiseFreq = 0.0f;

  float frameHarmonic1Amp = 0.0f;
  float frameHarmonic1Freq = 0.0f;

  float frameHarmonic2Amp = 0.0f;
  float frameHarmonic2Freq = 0.0f;

  float propDamage[4] {};
  float groundEffect[4] {};

  // charged battery voltage
  float vbat = 0.0f;

  // 1 = any contact active (summary of contactCount)
  uint8_t contact = 0;

  // contact manifold for this frame; persists across the frame's sub-ticks
  uint8_t contactCount = 0;
  ContactPoint contacts[6] {};
};

//simulation output for game
struct StateOutput{
  Vec4F orientation{};
  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  // rpm
  float motorRpm[4] {};
  // temperature
  float motorT[4] {};
  // status
  int32_t motorStatus[4] {};

  // effective per-motor damage 0..1 (max of client input and accumulated)
  float propDamage[4] {};

  // beeper on (1) / off (0)
  uint8_t beep = 0U;

  // bit0 sim crashed latch, bit1 BF crash recovery, bit2 BF turtle (reserved)
  uint8_t crashFlags = 0U;

  uint8_t osd[16*30] {};
};

#endif