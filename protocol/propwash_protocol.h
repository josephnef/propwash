/*
 * propwash wire protocol — the ONLY interface between propwash-core (GPL-3.0)
 * and its clients (Godot, Quest, Python gym, tools).
 *
 * This header is MIT-licensed (see LICENSE.MIT) and MUST NOT include any
 * Betaflight header — it is the license boundary.
 *
 * Encoding: packed little-endian structs, one packet per UDP datagram,
 * PwHeader first, payload immediately after.
 */

#ifndef PROPWASH_PROTOCOL_H
#define PROPWASH_PROTOCOL_H

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

#define PW_MAGIC   0x48535750u /* "PWSH" little-endian */
#define PW_VERSION 2u

#define PW_MOTOR_COUNT  4
#define PW_RC_CHANNELS  8
#define PW_OSD_ROWS     16
#define PW_OSD_COLS     30
#define PW_MAX_CONTACTS 6

/* Reference collision hull for the CineLog35 profile, sim frame (+z forward,
 * +y up), metres. Every sender (Godot, Python harnesses, pw-tester) derives
 * its contact manifolds from the same five spheres so the core sees one hull.
 * Resting on flat ground the body origin sits at PW_HULL_REST_H (the duct
 * spheres touch first: 0.030 - 0.010). */
#define PW_HULL_BELLY_X 0.0f
#define PW_HULL_BELLY_Y 0.030f
#define PW_HULL_BELLY_Z 0.0f
#define PW_HULL_BELLY_R 0.045f
#define PW_HULL_DUCT_XZ 0.054f  /* 4 duct spheres at (+-XZ, Y, +-XZ) */
#define PW_HULL_DUCT_Y  0.010f
#define PW_HULL_DUCT_R  0.030f
#define PW_HULL_REST_H  0.020f

#pragma pack(push, 1)

typedef enum {
    PW_INIT          = 1, /* client -> core : PwInit                       */
    PW_INIT_ACK      = 2, /* core -> client : PwInitAck                    */
    PW_STATE_IN      = 3, /* client -> core : PwStateIn (advances sim)     */
    PW_STATE_OUT     = 4, /* core -> client : PwStateOut (per PW_STATE_IN) */
    PW_OSD           = 5, /* core -> client : PwOsd (throttled)            */
    PW_RC_OVERRIDE   = 6, /* any peer -> core : PwRcOverride               */
    PW_COMMAND       = 7, /* client -> core : PwCommand                    */
} PwPacketType;

typedef enum {
    PW_CMD_RESET    = 1, /* re-init firmware + physics                    */
    PW_CMD_PAUSE    = 2,
    PW_CMD_RESUME   = 3,
    PW_CMD_REALTIME = 4, /* core self-ticks on wall clock                 */
    PW_CMD_LOCKSTEP = 5, /* core ticks only on PW_STATE_IN (default)      */
    PW_CMD_REPAIR   = 6, /* clear damage + recharge battery; pose untouched */
} PwCommandType;

/* Surface material of a contact, indexes the core's contact-material table
 * (stiffness/friction/restitution) and scales impact damage. */
typedef enum {
    PW_SURF_GROUND = 0,
    PW_SURF_GATE   = 1,
    PW_SURF_TREE   = 2,
    PW_SURF_OBJECT = 3,
} PwSurfaceType;

typedef enum {
    PW_MOTOR_OK        = 0,
    PW_MOTOR_BURNEDOUT = 1,
    PW_MOTOR_DAMAGED   = 2,
} PwMotorStatus;

typedef struct {
    uint32_t magic;       /* PW_MAGIC                                     */
    uint8_t  version;     /* PW_VERSION                                   */
    uint8_t  type;        /* PwPacketType                                 */
    uint16_t payload_len; /* bytes following this header                  */
} PwHeader;

typedef struct { float x, y, z; }    PwVec3;
typedef struct { float w, x, y, z; } PwQuat;

/* Physical profile of the quad. Mirrors the physics parameterisation
 * (SimITL lineage); loaded by the core from a JSON profile and/or sent by
 * the client at session start. */
typedef struct {
    float  motor_kv[PW_MOTOR_COUNT];
    float  motor_R[PW_MOTOR_COUNT];   /* winding resistance, ohm          */
    float  motor_I0[PW_MOTOR_COUNT];  /* idle current, A                  */

    uint8_t prop_blade_count;
    float  prop_max_rpm;
    float  prop_a_factor;
    float  prop_torque_factor;
    float  prop_inertia;
    PwVec3 prop_thrust_factor;        /* quadratic thrust vs airspeed     */

    PwVec3 frame_drag_area;           /* m^2 per axis                     */
    float  frame_drag_constant;       /* Cd                               */

    float  quad_mass;                 /* kg                               */
    PwVec3 quad_inv_inertia;          /* 1/(kg m^2), diagonal             */
    PwVec3 quad_motor_pos[PW_MOTOR_COUNT]; /* body frame, m               */

    uint8_t bat_cell_count;
    float  bat_capacity_mah;          /* rated                            */
    float  bat_capacity_charged_mah;  /* actual charged                   */
    float  max_voltage_sag;           /* V at full load                   */

    float  min_propwash_speed;        /* m/s                              */
    float  max_propwash_speed;        /* m/s                              */
    float  propwash_angle_of_attack;
    float  propwash_factor;

    uint32_t seed;                    /* noise RNG seed (determinism)     */
} PwInit;

typedef struct {
    uint8_t  protocol_version;
    char     fc_variant[5];           /* "BTFL", NUL-terminated           */
    char     fc_version[16];          /* e.g. "4.5.2"                     */
    char     core_version[32];        /* propwash-core git describe      */
} PwInitAck;

/* One contact point between the quad's hull and the world, detected by the
 * client (which owns geometry+position) and resolved by the core (which owns
 * dynamics). point_body is in the quad's body frame — the core has no world
 * position, and R*point_body is exactly the lever arm it needs. normal_world
 * is a unit vector pointing from the surface toward the quad. */
typedef struct {
    PwVec3  point_body;               /* contact point, body frame, m     */
    PwVec3  normal_world;             /* unit surface normal, world frame */
    float   depth;                    /* penetration at the sent pose, m  */
    uint8_t surface;                  /* PwSurfaceType                    */
} PwContact;

/* One simulation advance. The client owns collision *detection* and world
 * position: it sends the authoritative pose plus the contact manifold for
 * this frame. The core owns dynamics: as of v2 the velocity fields are
 * ignored (kept for layout/telemetry) — the core integrates its own
 * velocities and resolves contacts as forces inside the tick, so impacts
 * reach the firmware's virtual gyro/accel like the real sensors would. */
typedef struct {
    uint32_t frame_id;                /* echoed in PwStateOut             */
    float    dt;                      /* seconds to advance (clamped 0.1) */

    float    rc[PW_RC_CHANNELS];      /* normalised -1..1                 */

    PwVec3   position;                /* world, m                         */
    PwQuat   rotation;
    PwVec3   angular_velocity;        /* rad/s (ignored as of v2)         */
    PwVec3   linear_velocity;         /* m/s   (ignored as of v2)         */

    float    gyro_noise_amp;
    float    prop_damage[PW_MOTOR_COUNT];   /* 0..1, external/scripted    */
    float    ground_effect[PW_MOTOR_COUNT]; /* 0..1                       */
    float    vbat_charged;            /* charged pack voltage, V          */
    uint8_t  contact;                 /* 1 = any contact active (summary) */

    uint8_t  contact_count;           /* 0..PW_MAX_CONTACTS               */
    PwContact contacts[PW_MAX_CONTACTS];
} PwStateIn;

typedef struct {
    uint32_t frame_id;                /* echo of PwStateIn.frame_id       */
    uint64_t sim_time_us;             /* firmware-side simulated time     */

    PwQuat   orientation;
    PwVec3   angular_velocity;        /* rad/s                            */
    PwVec3   linear_velocity;         /* m/s                              */
    PwVec3   position;                /* world, m (core-integrated)       */
    PwVec3   acceleration;            /* m/s^2                            */

    float    motor_rpm[PW_MOTOR_COUNT];
    uint8_t  motor_status[PW_MOTOR_COUNT];  /* PwMotorStatus              */

    uint8_t  armed;                   /* firmware ARMED flag              */
    uint32_t arming_disable_flags;    /* firmware arming-disable bitmask  */
    uint32_t flight_mode_flags;       /* firmware flightModeFlags         */
    uint8_t  beeper;

    float    vbat;                    /* sagged voltage as firmware sees  */
    float    amperage;

    /* Effective per-motor damage (max of the client-sent prop_damage and
     * the core's impact-accumulated damage). */
    float    prop_damage[PW_MOTOR_COUNT];
    /* bit0 = sim structural-crash latch (cleared by REPAIR/RESET)
     * bit1 = firmware crash recovery active
     * bit2 = firmware flip-over-after-crash (turtle) active — reserved,
     *        unreachable on the SITL target today (needs DSHOT). */
    uint8_t  crash_flags;
} PwStateOut;

typedef struct {
    uint8_t chars[PW_OSD_ROWS * PW_OSD_COLS];
} PwOsd;

typedef struct {
    float rc[PW_RC_CHANNELS];         /* normalised -1..1                 */
} PwRcOverride;

typedef struct {
    uint32_t command;                 /* PwCommandType                    */
} PwCommand;

#pragma pack(pop)

#if defined(__cplusplus)
} /* extern "C" */
#endif

#endif /* PROPWASH_PROTOCOL_H */
