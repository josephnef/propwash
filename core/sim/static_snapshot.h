#ifndef PW_STATIC_SNAPSHOT_H
#define PW_STATIC_SNAPSHOT_H

#include <cstddef>

namespace SimITL {

  /* Deterministic reset by construction: capture every writable static
   * section of the main executable right after the first firmware boot, and
   * restore it on reset instead of trusting BF::init() to clear ~40
   * subsystems' worth of statics (it doesn't: scheduler task stats, IMU
   * quaternion, PID state, gyro calibration, RX/arming latches, OSD/CMS
   * timers, ... all survive an in-process re-init and made every reset — and
   * every session — start from a slightly different firmware state).
   *
   * Same-process restore is inherently pointer-safe (every captured pointer
   * value is valid in this process) EXCEPT where lifetime or concurrency
   * disagree — those ranges are registered as exclusions before the
   * snapshot: dyad's connection state (its fd arrays get realloc'd; a live
   * pump thread mutates it) and the dyad mutex (held during the restore).
   * The snapshot's own bookkeeping is excluded automatically. */

  // Exclude [addr, addr+size) from restore. Call before snapshotTake().
  void snapshotExclude(void* addr, size_t size);

  // Capture all writable static sections. First call wins; later calls are
  // no-ops (the canonical state is the FIRST boot). Returns success.
  bool snapshotTake();

  // Memcpy the captured sections back, skipping exclusions. Returns false
  // if no snapshot exists (or the platform walk is unsupported).
  bool snapshotRestore();

  bool snapshotTaken();

}

#endif // PW_STATIC_SNAPSHOT_H
