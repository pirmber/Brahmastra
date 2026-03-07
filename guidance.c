/*
 * guidance.c — Proportional Navigation and PID Guidance
 *
 * ═══════════════════════════════════════════════════════════════════════
 * PROPORTIONAL NAVIGATION
 * ═══════════════════════════════════════════════════════════════════════
 * The PN law generates lateral acceleration perpendicular to the missile
 * velocity to null the Line-of-Sight (LOS) rotation rate.
 *
 *   LOS angle:   λ  = atan2(Δy, Δx)  where Δ = target_pos - missile_pos
 *   LOS rate:    λ̇  ≈ (λ - λ_prev) / dt
 *   Closing vel: V_c = -d(range)/dt = -(R⃗ · V⃗_rel) / |R⃗|
 *   Command:     a_cmd = N * V_c * λ̇   (perpendicular to LOS)
 *
 * ═══════════════════════════════════════════════════════════════════════
 * PID GUIDANCE
 * ═══════════════════════════════════════════════════════════════════════
 * Computes angular error between missile heading and desired LOS bearing,
 * runs a PID controller on that error, and outputs lateral acceleration.
 *
 *   error       = angle_wrap(λ - missile_heading)
 *   a_lateral   = Kp*e + Ki*∫e dt + Kd*(de/dt)
 * ═══════════════════════════════════════════════════════════════════════
 */

#include "guidance.h"
#include <math.h>
#include <string.h>
#define M_PI 3.14159265358979323846
/* Wrap angle to [-π, π] */
static double wrap_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/* ── Proportional Navigation ──────────────────────────────────────── */

void pn_init(PNState *pn, double nav_constant)
{
    memset(pn, 0, sizeof(*pn));
    pn->nav_constant = nav_constant;
    pn->initialized  = 0;
}

Vec2 pn_compute(PNState *pn,
                Vec2 missile_pos, Vec2 missile_vel,
                Vec2 target_pos,  Vec2 target_vel,
                double dt)
{
    Vec2 rel_pos = vec2_sub(target_pos, missile_pos); /* R⃗ */
    Vec2 rel_vel = vec2_sub(target_vel, missile_vel); /* V⃗_rel */

    double range = vec2_mag(rel_pos);
    if (range < 1e-3) return vec2_zero();

    /* LOS angle */
    double los_angle = atan2(rel_pos.y, rel_pos.x);

    /* First-step initialization */
    if (!pn->initialized) {
        pn->prev_los_angle = los_angle;
        pn->initialized    = 1;
        return vec2_zero();
    }

    /* LOS rotation rate λ̇ */
    double los_rate = wrap_angle(los_angle - pn->prev_los_angle) / dt;
    pn->prev_los_angle = los_angle;

    /* Closing velocity: V_c = -(R⃗ · V⃗_rel) / |R⃗| */
    double v_closing = -vec2_dot(rel_pos, rel_vel) / range;

    /* Commanded lateral acceleration magnitude */
    double a_cmd = pn->nav_constant * v_closing * los_rate;

    /*
     * Acceleration direction is perpendicular to LOS:
     * rotate LOS unit vector 90 degrees
     */
    Vec2 los_unit = vec2_normalize(rel_pos);
    Vec2 perp     = { -los_unit.y, los_unit.x };

    return vec2_scale(perp, a_cmd);
}

/* ── PID Guidance ─────────────────────────────────────────────────── */

void pid_init(PIDState *pid, double kp, double ki, double kd)
{
    memset(pid, 0, sizeof(*pid));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

Vec2 pid_compute(PIDState *pid,
                 Vec2 missile_pos, Vec2 missile_vel,
                 Vec2 target_pos,
                 double max_accel, double dt)
{
    Vec2   rel_pos    = vec2_sub(target_pos, missile_pos);
    double los_angle  = atan2(rel_pos.y, rel_pos.x);
    double msl_angle  = vec2_angle(missile_vel);
    double error      = wrap_angle(los_angle - msl_angle);

    /* PID terms */
    pid->integral  += error * dt;
    double deriv    = pid->initialized
                      ? (error - pid->prev_error) / dt
                      : 0.0;
    pid->prev_error = error;
    pid->initialized = 1;

    double a_lateral = pid->kp * error
                     + pid->ki * pid->integral
                     + pid->kd * deriv;

    /* Apply perpendicular to current missile heading */
    Vec2 msl_dir = vec2_normalize(missile_vel);
    Vec2 perp    = { -msl_dir.y, msl_dir.x };

    Vec2 accel = vec2_scale(perp, a_lateral);
    return vec2_clamp_mag(accel, max_accel);
}

/* ── Unified Interface ────────────────────────────────────────────── */

void guidance_init(GuidanceSystem *gs, GuidanceMode mode,
                   double max_accel,
                   double pn_nav_constant,
                   double pid_kp, double pid_ki, double pid_kd)
{
    memset(gs, 0, sizeof(*gs));
    gs->mode      = mode;
    gs->max_accel = max_accel;
    pn_init(&gs->pn, pn_nav_constant);
    pid_init(&gs->pid, pid_kp, pid_ki, pid_kd);
}

Vec2 guidance_compute(GuidanceSystem *gs,
                      Vec2 missile_pos, Vec2 missile_vel,
                      Vec2 target_pos,  Vec2 target_vel,
                      double dt)
{
    Vec2 cmd = vec2_zero();

    switch (gs->mode) {
        case GUIDANCE_PN:
            cmd = pn_compute(&gs->pn,
                             missile_pos, missile_vel,
                             target_pos,  target_vel, dt);
            break;

        case GUIDANCE_PID:
            cmd = pid_compute(&gs->pid,
                              missile_pos, missile_vel,
                              target_pos,
                              gs->max_accel, dt);
            break;
    }

    return vec2_clamp_mag(cmd, gs->max_accel);
}

const char *guidance_mode_name(GuidanceMode mode)
{
    switch (mode) {
        case GUIDANCE_PN:  return "Proportional Navigation (PN)";
        case GUIDANCE_PID: return "PID Heading Correction";
        default:           return "Unknown";
    }
}
