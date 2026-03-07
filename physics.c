/*
 * physics.c — Physics Body Integration
 *
 * Uses semi-implicit Euler integration:
 *   vel += accel * dt
 *   pos += vel * dt
 * Simple, stable for small dt values (dt <= 0.01s).
 */

#include "physics.h"
#include <string.h>

void physics_init(PhysicsBody *body,
                  Vec2 pos, Vec2 vel,
                  double max_speed, double max_accel,
                  double drag_coeff, double mass)
{
    memset(body, 0, sizeof(*body));
    body->pos        = pos;
    body->vel        = vel;
    body->accel      = vec2_zero();
    body->max_speed  = max_speed;
    body->max_accel  = max_accel;
    body->drag_coeff = drag_coeff;
    body->mass       = (mass > 0.0) ? mass : 1.0;
    body->heading    = vec2_angle(vel);
    body->speed      = vec2_mag(vel);
}

void physics_apply_accel(PhysicsBody *body, Vec2 accel)
{
    /* Clamp to max acceleration magnitude */
    body->accel = vec2_clamp_mag(accel, body->max_accel);
}

void physics_step(PhysicsBody *body, double dt)
{
    /* 1. Apply aerodynamic drag: F_drag = -drag_coeff * v * |v| */
    if (body->drag_coeff > 0.0) {
        double spd = vec2_mag(body->vel);
        if (spd > 1e-6) {
            Vec2 drag_force = vec2_scale(body->vel,
                                         -body->drag_coeff * spd / body->mass);
            body->accel = vec2_add(body->accel, drag_force);
        }
    }

    /* 2. Semi-implicit Euler: update velocity then position */
    body->vel = vec2_add(body->vel, vec2_scale(body->accel, dt));

    /* 3. Enforce speed limit */
    body->vel = vec2_clamp_mag(body->vel, body->max_speed);

    /* 4. Update position */
    body->pos = vec2_add(body->pos, vec2_scale(body->vel, dt));

    /* 5. Update derived state */
    body->speed   = vec2_mag(body->vel);
    body->heading = vec2_angle(body->vel);

    /* 6. Clear acceleration for next frame */
    body->accel = vec2_zero();
}
