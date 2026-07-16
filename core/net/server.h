/**
 * UDP protocol server — the only interface between propwash-core and its
 * clients (Godot, Quest, gym, tools). Single client at a time (last sender
 * of PW_INIT/PW_STATE_IN wins). Lockstep: each PW_STATE_IN advances the
 * simulation and is answered with one PW_STATE_OUT (echoed frame_id).
 */

#ifndef PW_SERVER_H
#define PW_SERVER_H

#include <cstdint>

#include "propwash_protocol.h"
#include "sim/sim.h"
#include "input/joystick.h"

namespace pw {

class Server {
  public:
    /** Bind the UDP port. \return true on success. */
    bool start(uint16_t port);

    /**
     * Serve forever (blocking). BF is booted on the first PW_INIT (or with
     * the built-in CineLog35 profile on the first PW_STATE_IN if the client
     * never sends one).
     */
    void run(SimITL::Sim& sim, const StateInit& defaultInit, Joystick* js);

  private:
    void sendTo(const void* payload, uint16_t len, uint8_t type);

    int mFd = -1;
    // last known client
    uint32_t mClientAddr = 0;
    uint16_t mClientPort = 0;
    bool mHaveClient = false;
};

} // namespace pw

#endif // PW_SERVER_H
