"""Excitation maneuvers for record / replay / sysid.

Motor maneuvers (open-loop, for PW_MOTOR_IN physics-only replay) are the clean
signal for a physics fit: we command the motors, so the resulting gyro/accel
trajectory is a pure function of the physics parameters.

RC maneuvers drive the firmware (PW_STATE_IN) the way the Godot client does.
"""


def collective_climb(dt, seconds=1.6, ramp=0.4, level=0.55):
    """All four motors together: ramp up over `ramp` s to `level`, then hold.
    A straight vertical climb — the vertical accel/altitude trajectory it
    produces identifies thrust (and, with an independent signal, mass)."""
    n = int(seconds / dt)
    seq = []
    for i in range(n):
        t = i * dt
        c = level * min(1.0, t / ramp) if ramp > 0 else level
        seq.append((c, c, c, c))
    return seq


def roll_doublet(dt, seconds=1.6, level=0.5, amp=0.12, t0=0.5, t1=1.0):
    """Collective hold with a left/right differential pulse between t0 and t1 —
    excites a roll rate, so the gyro response identifies roll inertia/torque.
    Motor index layout matches profile quad_motor_pos (0,1 on +x; 2,3 on -x)."""
    n = int(seconds / dt)
    seq = []
    for i in range(n):
        t = i * dt
        d = amp if t0 <= t < t1 else 0.0
        seq.append((level + d, level + d, level - d, level - d))
    return seq


def rc_hover_climb(arm_t=5.4, angle=True):
    """RC function t -> 8 channels: arm after `arm_t` (gyro-cal window), then a
    gentle collective climb in ANGLE mode. AETR + aux5=ARM, aux6=ANGLE."""
    def fn(t):
        armed = t >= arm_t
        thr = -1.0
        if armed:
            # ramp throttle to a hover-ish level after arming
            thr = min(0.15, -0.2 + 0.25 * (t - arm_t))
        return [0.0, 0.0, thr, 0.0,
                1.0 if armed else -1.0,
                1.0 if angle else -1.0,
                -1.0, -1.0]
    return fn
