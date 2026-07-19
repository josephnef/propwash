"""Flight-log schema shared by record / replay / sysid, plus a best-effort
importer for real Betaflight blackbox logs (blackbox_decode CSV).

A log is a list of frame dicts with SI units:
  t                seconds
  rc0..rc7         normalised RC -1..1 (what the pilot/maneuver commanded)
  mot0..mot3       per-motor ESC command 0..1 (what the FC output)
  gx,gy,gz         gyro, rad/s (sim frame)
  ax,ay,az         accel, m/s^2 (sim frame, gravity included as the FC sees it)
  qw,qx,qy,qz      orientation quaternion
  vx,vy,vz         linear velocity, m/s
  px,py,pz         world position, m
  vbat             pack voltage, V

The native format (write_log/read_log) is what the sim records and replays.
import_betaflight_csv maps a real decoded log onto the same schema so a genuine
CineLog35 flight drops into the same replay/sysid path once the quad has flown.
"""
import csv
import math

FIELDS = (
    ["t"]
    + [f"rc{i}" for i in range(8)]
    + [f"mot{i}" for i in range(4)]
    + ["gx", "gy", "gz", "ax", "ay", "az"]
    + ["qw", "qx", "qy", "qz"]
    + ["vx", "vy", "vz", "px", "py", "pz", "vbat"]
)

# fields compared by default when scoring a replay against a reference
GYRO = ["gx", "gy", "gz"]
ACCEL = ["ax", "ay", "az"]
MOTOR_RPM = None  # rpm isn't in the schema; compare gyro/accel (the FC's sensors)


def write_log(path, frames):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for fr in frames:
            w.writerow({k: fr.get(k, 0.0) for k in FIELDS})


def read_log(path):
    frames = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            frames.append({k: float(row[k]) for k in FIELDS if k in row})
    return frames


def rmse(ref, sim, fields):
    """Root-mean-square error between two logs over the given fields, aligned by
    index (both come from the same maneuver, same frame count). Truncates to the
    shorter of the two if they differ."""
    n = min(len(ref), len(sim))
    if n == 0:
        return float("inf")
    acc = 0.0
    for i in range(n):
        for k in fields:
            d = ref[i].get(k, 0.0) - sim[i].get(k, 0.0)
            acc += d * d
    return math.sqrt(acc / (n * len(fields)))


def per_field_rmse(ref, sim, fields):
    n = min(len(ref), len(sim))
    out = {}
    for k in fields:
        acc = sum((ref[i].get(k, 0.0) - sim[i].get(k, 0.0)) ** 2 for i in range(n))
        out[k] = math.sqrt(acc / n) if n else float("inf")
    return out


# --- real Betaflight blackbox import --------------------------------------

DEG2RAD = math.pi / 180.0


def _col(row, *names, default=0.0):
    for n in names:
        if n in row and row[n] not in ("", None):
            return float(row[n])
    return default


def import_betaflight_csv(path, acc_1g=2048.0, motor_min=1000.0,
                          motor_max=2000.0):
    """Map a `blackbox_decode` CSV onto the native schema. Best-effort — real
    logs vary by firmware/version and by the flags passed to blackbox_decode:

    - gyro:  gyroADC[0..2] are deg/s -> rad/s.
    - accel: accSmooth[0..2] are in units of acc_1G (raw); scaled to m/s^2 with
             `acc_1g` (Betaflight default 2048). If the log was decoded with a
             unit flag, columns already in m/s/s or G are detected and used.
    - motor: motor[0..3] normalised from [motor_min, motor_max] to 0..1
             (classic PWM range; pass DSHOT bounds e.g. 48/2047 if needed).
    - rc:    rcCommand[0..2] (-500..500) -> -1..1; rcCommand[3] throttle
             (1000..2000) -> -1..1.
    Orientation/velocity/position are not in a standard blackbox log; they are
    left at zero — replay in RC mode reconstructs them, and physics-only (motor)
    replay does not need them. Gyro + accel are what a physics fit scores on.
    """
    frames = []
    with open(path, newline="") as f:
        # blackbox_decode CSV often has a space after each comma
        reader = csv.DictReader(f, skipinitialspace=True)
        t0 = None
        for row in reader:
            row = {k.strip(): v for k, v in row.items() if k}
            t_us = _col(row, "time", "time (us)")
            if t0 is None:
                t0 = t_us
            fr = {k: 0.0 for k in FIELDS}
            fr["t"] = (t_us - t0) * 1e-6
            fr["gx"] = _col(row, "gyroADC[0]") * DEG2RAD
            fr["gy"] = _col(row, "gyroADC[1]") * DEG2RAD
            fr["gz"] = _col(row, "gyroADC[2]") * DEG2RAD
            for i, axis in enumerate("xyz"):
                ms2 = _col(row, f"accSmooth[{i}] (m/s/s)", default=None) \
                    if f"accSmooth[{i}] (m/s/s)" in row else None
                if ms2 is not None:
                    fr["a" + axis] = ms2
                else:
                    g = _col(row, f"accSmooth[{i}] (G)", default=None) \
                        if f"accSmooth[{i}] (G)" in row else None
                    raw = _col(row, f"accSmooth[{i}]")
                    fr["a" + axis] = (g * 9.80665) if g is not None \
                        else (raw / acc_1g * 9.80665)
            span = max(1.0, motor_max - motor_min)
            for i in range(4):
                fr[f"mot{i}"] = min(1.0, max(0.0,
                                    (_col(row, f"motor[{i}]") - motor_min) / span))
            for i in range(3):
                fr[f"rc{i}"] = max(-1.0, min(1.0, _col(row, f"rcCommand[{i}]") / 500.0))
            thr = _col(row, "rcCommand[3]", default=1000.0)
            fr["rc2"] = max(-1.0, min(1.0, (thr - 1500.0) / 500.0))
            fr["vbat"] = _col(row, "vbatLatest (V)", "vbatLatest", default=16.8)
            frames.append(fr)
    return frames
