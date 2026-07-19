#pragma once
/*
 * propwash: sim-safe replacement for build/atomic.h, found ahead of the
 * stock header via the simstubs/ include dir (same shadowing pattern as the
 * target.h and dyad.h wrappers).
 *
 * The stock header masks Cortex interrupts by writing BASEPRI with inline
 * ARM assembly — meaningless off-target and uncompilable on x86/arm64
 * hosts. The sim runs the entire firmware on one thread (dyad's pump is
 * serialised separately via pw_dyad_lock), so a critical section reduces to
 * a compiler barrier: code shape and ordering are preserved, with no ARM
 * dependency. Equivalent in spirit to the stock header's UNIT_TEST branch.
 */
#include <stdint.h>

static inline void pwAtomicBarrier(void)
{
    __asm__ volatile ("" ::: "memory");
}

#define ATOMIC_BLOCK(prio) \
    for (uint8_t pw_atomic_once = (pwAtomicBarrier(), 1); pw_atomic_once; \
         pw_atomic_once = (pwAtomicBarrier(), 0))

#define ATOMIC_BLOCK_NB(prio) ATOMIC_BLOCK(prio)

#define ATOMIC_BARRIER(data) pwAtomicBarrier()
