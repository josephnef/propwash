/**
 * Simulation loop — ported from SimITL's sim.h/sim.cpp (GPL-3.0), stripped
 * of the shared-memory/websocket frontends (propwash uses a UDP protocol
 * server in core/net instead).
 */

#ifndef PW_SIM_H
#define PW_SIM_H

#include <cstdint>

#include "bf.h"
#include "state.h"
#include "physics.h"

namespace SimITL{

  class Sim {
    public:
      static Sim& getInstance();

      /** Initialise physics from profile and boot the firmware. */
      void init(const StateInit& stateInit);

      /** Re-initialise the physics only (crash reset). */
      void reinitPhysics(const StateInit& stateInit);

      /* Full reset: firmware re-init, physics, noise RNG and the sub-tick
       * accumulator. PW_CMD_RESET used to restore only a fraction of this, so
       * a reset run did not match a freshly started one. */
      void reset(const StateInit& stateInit);

      /* Physics, noise RNG and sub-tick accumulator only — no firmware touch.
       * Used at first client contact, where re-initing the firmware could
       * disturb an already-attached Configurator session. */
      void resetPhysicsOnly(const StateInit& stateInit);

      /**
       * Advance the simulation by stateInput.delta seconds, in fixed
       * DELTA-sized firmware/physics ticks (accumulator).
       */
      void update(const StateInput& stateInput);

      const StateOutput& getStateUpdate() const;
      const SimState& getSimState() const { return mSimState; }

      void command(const CommandType cmd);

    private:
      Sim();
      ~Sim();

      void simStep();

      SimState mSimState {};
      Physics mPhysics {};
      int64_t total_delta = 0;
  };

}

#endif // PW_SIM_H
