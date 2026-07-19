/**
 * Access to Betaflight 4.5.2 internals from C++.
 * Pattern ported from SimITL's sitl.h (GPL-3.0): C headers are included
 * inside a namespace with extern "C" linkage, so all firmware symbols are
 * reachable as SimITL::BF::* while linking against the C static library.
 *
 * Standard headers are pre-included at global scope so their include
 * guards prevent re-emission inside the namespace.
 */

#ifndef PW_BFBRIDGE_H
#define PW_BFBRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <limits.h>
#include <float.h>
#include <inttypes.h>

namespace SimITL{
  namespace BF {
    extern "C" {
      #include "platform.h"

      #include "common/maths.h"

      #include "fc/init.h"
      #include "fc/runtime_config.h"
      #include "fc/tasks.h"
      #include "fc/core.h"

      #include "flight/imu.h"

      #include "scheduler/scheduler.h"
      #include "sensors/sensors.h"

      #include "drivers/accgyro/accgyro_virtual.h"
      #include "drivers/pwm_output.h"
      #include "drivers/motor.h"
      #include "drivers/dshot_command.h"

      #include "rx/rx.h"
      #include "rx/msp.h"

      #include "fc/rc_modes.h"
      #include "flight/pid.h"
      #include "io/beeper.h"

      #include "build/debug.h"

      // OSD character grid captured by the fake max7456 displayport
      #include "displayport_fake.h"

      #undef ENABLE_STATE

      //custom macro with bf namespaces
      #define BF_DEBUG_SET(mode, index, value) do { if (BF::debugMode == (mode)) { BF::debug[(index)] = (value); } } while (0)

      // rc data
      static uint16_t rcDataCache[SIMULATOR_MAX_RC_CHANNELS] {};
      static uint32_t rcDataReceptionTimeUs = 0U;

      static float rxRcReadData(const BF::rxRuntimeState_t *rxRuntimeState, uint8_t channel)
      {
        UNUSED(rxRuntimeState);
        return rcDataCache[channel];
      }

      static uint32_t pw_rc_frame_status_calls = 0;

      static uint8_t rxRcFrameStatus(BF::rxRuntimeState_t *rxRuntimeState)
      {
        UNUSED(rxRuntimeState);
        pw_rc_frame_status_calls++;
        return BF::RX_FRAME_COMPLETE;
      }

      // propwash target globals (pw_sitl.c)
      extern uint64_t pw_micros_passed;
      extern int64_t  pw_sleep_timer;
      extern int16_t  pw_motors_pwm[];
      // per-motor spin direction from the virtual DSHOT ESC (+1 / -1)
      extern int8_t   pw_motor_dir[];
      void pw_set_eeprom_path(const char *path);

      // dyad serialisation + the address ranges the deterministic-reset
      // snapshot must exclude (dyad's live connection state; the mutex the
      // restore itself holds) — pw_sitl.c and the vendored dyad.c
      void pw_dyad_lock(void);
      void pw_dyad_unlock(void);
      void pw_dyad_state_range(void **addr, unsigned long *size);
      void pw_dyad_mutex_range(void **addr, unsigned long *size);

    } // end extern "C"
  }
}
#endif // PW_BFBRIDGE_H
