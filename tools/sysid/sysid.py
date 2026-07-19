#!/usr/bin/env python3
"""System identification: fit physics parameters so the sim matches a reference
blackbox log.

  sysid.py ref.csv --fit quad_mass,prop_thrust_factor.2 --iters 40

Uses physics-only (motor) replay: the reference log's ESC commands are fed
through the physics for each candidate parameter set, and the parameters are
adjusted to minimise the gyro+accel RMSE against the log. Evaluations reuse one
core (PW_INIT re-parameterises physics live), so a fit is a couple of minutes,
not hundreds of process spawns.

Importable: fit(core, ref, keys, ...) returns (best_params, history) for tests.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import bblog  # noqa: E402
import profiles  # noqa: E402
from runner import Session  # noqa: E402

DEFAULT_CORE = os.path.join(os.path.dirname(__file__), "..", "..",
                            "build", "propwash-core")
SCORE_FIELDS = bblog.GYRO + bblog.ACCEL


def hooke_jeeves(f, x0, bounds, step0=0.25, shrink=0.5, tol=1e-3, max_evals=200):
    """Pattern search (derivative-free), robust for smooth low-dim fits. x and
    steps live in normalised [0,1] space per bounds so dims are comparable."""
    n = len(x0)
    lo = [b[0] for b in bounds]
    hi = [b[1] for b in bounds]

    def to_norm(x):
        return [(x[i] - lo[i]) / (hi[i] - lo[i]) for i in range(n)]

    def to_real(u):
        return [lo[i] + min(1.0, max(0.0, u[i])) * (hi[i] - lo[i]) for i in range(n)]

    evals = [0]

    def fn(u):
        evals[0] += 1
        return f(to_real(u))

    u = to_norm(x0)
    best = fn(u)
    step = step0
    while step > tol and evals[0] < max_evals:
        improved = False
        base = list(u)
        base_val = best
        for i in range(n):
            for s in (step, -step):
                cand = list(u)
                cand[i] = min(1.0, max(0.0, cand[i] + s))
                v = fn(cand)
                if v < best:
                    best, u, improved = v, cand, True
                    break
        if improved:
            # pattern move: extrapolate along the successful direction
            patt = [min(1.0, max(0.0, u[i] + (u[i] - base[i]))) for i in range(n)]
            v = fn(patt)
            if v < best:
                best, u = v, patt
            _ = base_val
        else:
            step *= shrink
    return to_real(u), best, evals[0]


def make_objective(session, ref, keys, dt, base=None):
    base = base or profiles.CINELOG35
    motors = [(f["mot0"], f["mot1"], f["mot2"], f["mot3"]) for f in ref]

    def objective(x):
        p = profiles.clone(base)
        for k, v in zip(keys, x):
            profiles.set_param(p, k, v)
        session.set_profile(p)
        sim = session.run_motor(motors, dt)
        return bblog.rmse(ref, sim, SCORE_FIELDS)
    return objective


def fit(core, ref, keys, x0=None, iters=200, session=None):
    """Fit `keys` against reference log `ref`. Returns (params_dict, rmse,
    n_evals). Reuses `session` if given (else spawns and closes one)."""
    dt = ref[1]["t"] - ref[0]["t"]
    bounds = [profiles.FITTABLE[k] for k in keys]
    if x0 is None:
        x0 = [0.5 * (b[0] + b[1]) for b in bounds]

    own = session is None
    s = session or Session(core)
    try:
        obj = make_objective(s, ref, keys, dt)
        best_x, best_val, n = hooke_jeeves(obj, x0, bounds, max_evals=iters)
    finally:
        if own:
            s.close()

    p = profiles.clone(profiles.CINELOG35)
    for k, v in zip(keys, best_x):
        profiles.set_param(p, k, v)
    return p, best_val, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref")
    ap.add_argument("--core", default=DEFAULT_CORE)
    ap.add_argument("--fit", default="quad_mass,prop_thrust_factor.2")
    ap.add_argument("--iters", type=int, default=120)
    args = ap.parse_args()

    keys = [k.strip() for k in args.fit.split(",") if k.strip()]
    for k in keys:
        if k not in profiles.FITTABLE:
            print(f"FAIL: '{k}' is not fittable; choose from "
                  f"{', '.join(profiles.FITTABLE)}")
            return 1

    ref = bblog.read_log(args.ref)
    p, err, n = fit(args.core, ref, keys, iters=args.iters)
    print(f"fit {keys} in {n} evals, final RMSE {err:.4f}")
    for k in keys:
        print(f"  {k:28s} = {profiles.get_param(p, k):.6g}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
