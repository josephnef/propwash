"""Physics profile: the PwInit parameter vector for system identification.

CINELOG35 mirrors the compiled baseline in core/sim/profile_cinelog35.h — the
subset of fields that Server::applyInit() actually writes (the rest keep the
core's built-in defaults). record.py sends this to pin the reference run to a
known profile; sysid.py perturbs a few entries and fits them back.

Field order here is exactly the C PwInit struct order — see wire.pack_init().
"""

# GEPRC CineLog35 V3, matching core/sim/profile_cinelog35.h.
CINELOG35 = {
    "motor_kv": [2650.0] * 4,
    "motor_R": [0.09] * 4,
    "motor_I0": [0.007] * 4,
    "prop_blade_count": 3,
    "prop_max_rpm": 33000.0,
    "prop_a_factor": 1.6e-9,
    "prop_torque_factor": 0.0070,
    "prop_inertia": 1.6e-7,
    "prop_thrust_factor": [-0.000004, -0.05, 2.8],
    "frame_drag_area": [0.0110, 0.0135, 0.0110],
    "frame_drag_constant": 1.35,
    "quad_mass": 0.33,
    "quad_inv_inertia": [900.0, 1100.0, 900.0],
    "quad_motor_pos": [
        [0.054, 0.005, -0.054],
        [0.054, 0.005, 0.054],
        [-0.054, 0.005, -0.054],
        [-0.054, 0.005, 0.054],
    ],
    "bat_cell_count": 4,
    "bat_capacity_mah": 660.0,
    "bat_capacity_charged_mah": 660.0,
    "max_voltage_sag": 1.6,
    "min_propwash_speed": 0.8,
    "max_propwash_speed": 10.0,
    "propwash_angle_of_attack": 0.1,
    "propwash_factor": 1.2,
    "seed": 0,
}

# Scalar parameters the fitter can identify, with plausible bounds. Vector
# fields (thrust_factor, inertia) are addressed by an index-suffixed key, e.g.
# "prop_thrust_factor.2" is the static max-thrust term (N per prop) — the one
# that, with quad_mass, dominates the vertical (open-loop) response.
FITTABLE = {
    "quad_mass": (0.15, 0.60),
    "prop_thrust_factor.2": (1.5, 4.5),
    "frame_drag_constant": (0.5, 2.5),
    "prop_torque_factor": (0.002, 0.020),
    "quad_inv_inertia.0": (400.0, 1800.0),
    "quad_inv_inertia.1": (400.0, 1800.0),
}


def clone(profile):
    """Deep-ish copy (lists one level down) so edits don't alias the baseline."""
    out = {}
    for k, v in profile.items():
        out[k] = [x[:] if isinstance(x, list) else x for x in v] \
            if isinstance(v, list) else v
    return out


def get_param(profile, key):
    """Read a scalar or an index-suffixed vector element (e.g. 'x.2')."""
    if "." in key:
        base, idx = key.split(".")
        return profile[base][int(idx)]
    return profile[key]


def set_param(profile, key, value):
    if "." in key:
        base, idx = key.split(".")
        profile[base][int(idx)] = value
    else:
        profile[key] = value
