/*
 * guidance.h — Missile Guidance Algorithms
 *
 * Implements two guidance laws:
 *
 * 1. PROPORTIONAL NAVIGATION (PN)
 *    Lateral acceleration = N * V_c * λ̇
 *    where:
 *      N    = navigation constant (typically 3–5)
 *      V_c  = closing velocity (rate of range decrease)
 *      λ̇   = line-of-sight (LOS) rotation rate (rad/s)
 *    The missile accelerates perpendicular to the LOS to null its rotation.
 *
 * 2. PID GUIDANCE
 *    Computes heading error between current velocity angle and desired
 *    LOS angle, then applies a PID controller to reduce that error.
 *    Output is a corrective lateral acceleration.
 */

#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "vec2.h"

typedef enum {
    GUIDANCE_PN  = 0,   /* Proportional Navigation */
    GUIDANCE_PID = 1    /* PID heading correction  */
} GuidanceMode;

/* ── Proportional Navigation State ─────────────────────────────────── */
typedef struct {
    double nav_constant;      /* N — navigation gain (3–5 typical)     */
    double prev_los_angle;    /* Previous LOS angle for λ̇ estimate     */
    int    initialized;       /* Whether prev_los_angle is set          */
} PNState;

void pn_init(PNState *pn, double nav_constant);

/* Returns required acceleration vector */
Vec2 pn_compute(PNState *pn,
                Vec2 missile_pos, Vec2 missile_vel,
                Vec2 target_pos,  Vec2 target_vel,
                double dt);

/* ── PID Guidance State ─────────────────────────────────────────────── */
typedef struct {
    double kp;            /* Proportional gain            */
    double ki;            /* Integral gain                */
    double kd;            /* Derivative gain              */
    double integral;      /* Accumulated error            */
    double prev_error;    /* Previous error for derivative */
    int    initialized;
} PIDState;

void pid_init(PIDState *pid, double kp, double ki, double kd);

/* Returns required acceleration vector */
Vec2 pid_compute(PIDState *pid,
                 Vec2 missile_pos, Vec2 missile_vel,
                 Vec2 target_pos,
                 double max_accel, double dt);

/* ── Unified Guidance Interface ─────────────────────────────────────── */
typedef struct {
    GuidanceMode mode;
    PNState      pn;
    PIDState     pid;
    double       max_accel;
} GuidanceSystem;

void guidance_init(GuidanceSystem *gs, GuidanceMode mode,
                   double max_accel,
                   double pn_nav_constant,
                   double pid_kp, double pid_ki, double pid_kd);

Vec2 guidance_compute(GuidanceSystem *gs,
                      Vec2 missile_pos, Vec2 missile_vel,
                      Vec2 target_pos,  Vec2 target_vel,
                      double dt);

const char *guidance_mode_name(GuidanceMode mode);

#endif /* GUIDANCE_H */
