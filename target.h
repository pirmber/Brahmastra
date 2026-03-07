/*
 * target.h — Moving Target Simulation
 *
 * Supports multiple motion patterns:
 *   TARGET_LINEAR     — Constant velocity
 *   TARGET_SINUSOIDAL — Lateral sinusoidal weave
 *   TARGET_EVASIVE    — Random periodic course changes
 */

#ifndef TARGET_H
#define TARGET_H

#include "physics.h"

typedef enum {
    TARGET_LINEAR     = 0,
    TARGET_SINUSOIDAL = 1,
    TARGET_EVASIVE    = 2
} TargetMotionMode;

typedef struct {
    PhysicsBody      body;
    TargetMotionMode mode;

    /* Sinusoidal parameters */
    double   sin_amplitude;   /* Lateral amplitude (m/s)    */
    double   sin_frequency;   /* Frequency (Hz)             */

    /* Evasive parameters */
    double   evasive_timer;   /* Time until next maneuver   */
    double   evasive_period;  /* Maneuver interval (s)      */
    Vec2     base_vel;        /* Base velocity direction    */
    double   evasive_angle;   /* Current evasion angle      */

    double   elapsed;         /* Total time elapsed (s)     */
} Target;

/* Initialize target */
void target_init(Target *t, Vec2 pos, Vec2 vel,
                 TargetMotionMode mode, double max_speed);

/* Step target forward by dt seconds */
void target_step(Target *t, double dt);

const char *target_mode_name(TargetMotionMode mode);

#endif /* TARGET_H */
