/**
 * Abstraction of betaflight functions.
 * Ported from SimITL (GPL-3.0) and adapted to Betaflight 4.5.2.
 */

#include <cstdint>
#include "state.h"

#ifndef BF_H
#define BF_H

#define E_DEBUG_SIM 90

namespace SimITL{
  namespace BF{
    extern "C" {
      // betaflight's init
      extern void init(void);
      extern void scheduler(void);
    }

    /**
     * \brief Resets rc channels to default values.
     * Prevents initial random channel states.
     */
    void resetRcData();

    /**
     * \brief Sets normalised RC channel data (-1..1) for the virtual FC.
     */
    void setRcData(const float (&data)[8]);

    /**
     * \brief Sets the eeprom file path the virtual FC reads/writes.
     */
    void setEepromFileName(const char* filename = "eeprom.bin");

    /**
     * \brief Updates the virtual fc's data with the provided state,
     * performs a betaflight schedule or sleeps and updates the state.
     * \param[in] dt Time delta in micro seconds.
     * \param[in,out] simState The state exchanged with the fc.
     * \return True if scheduler was executed, false if slept.
     */
    bool update(uint64_t dt, SimState& simState);

    /**
     * \brief BF debug call. Writes to blackbox debug fields.
     */
    void setDebugValue(uint8_t mode, uint8_t index, int16_t value);

    /**
     * \brief Configure ARM (AUX1/ch5) and ANGLE (AUX2/ch6) modes in RAM,
     * matching the CineLog35 switch plan. Call after init().
     */
    void configureDefaultModes();

    /** Bench mode: disable runaway-takeoff prevention (see bf.cpp). */
    void disableRunawayTakeoff();

    /**
     * \brief Capture the canonical firmware memory state (writable statics of
     * the whole process, with dyad's live state excluded). Call once, right
     * after the FIRST init() — before any simulated time has passed.
     */
    void takeStateSnapshot();

    /**
     * \brief Rewind the firmware statics to the snapshot (under the dyad
     * lock). The caller re-runs init() and the RAM-mode setup afterwards.
     * \return False when no snapshot exists (unsupported platform).
     */
    bool restoreStateSnapshot();
  }
}

#endif // BF_H
