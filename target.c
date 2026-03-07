/*
 * target.c — Moving Target Simulation
 */

#include "target.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define M_PI 3.14159265358979323846
void target_init(Target *t, Vec2 pos, Vec2 vel,
                 TargetMotionMode mode, double max_speed)
{
    memset(t, 0, sizeof(*t));
    physics_init(&t->body, pos, vel, max_speed, 9999.0, 0.0, 1.0);
    t->mode          = mode;
    t->elapsed       = 0.0;
    t->base_vel      = vel;

    /* Sinusoidal defaults */
    t->sin_amplitude  = max_speed * 0.8;
    t->sin_frequency  = 0.15;

    /* Evasive defaults */
    t->evasive_period = 3.5;
    t->evasive_timer  = t->evasive_period;
    t->evasive_angle  = 0.0;
}

void target_step(Target *t, double dt)
{
    t->elapsed += dt;

    Vec2 vel = t->base_vel;

    switch (t->mode) {

        case TARGET_LINEAR:
            /* Nothing extra — constant velocity */
            break;

        case TARGET_SINUSOIDAL: {
            /*
             * Add lateral sinusoidal component perpendicular to base heading.
             * Lateral direction = rotate base_vel 90 degrees.
             */
            double base_angle = vec2_angle(t->base_vel);
            Vec2 lateral = { -sin(base_angle), cos(base_angle) };
            double lateral_speed = t->sin_amplitude *
                                   sin(2.0 * M_PI * t->sin_frequency * t->elapsed);
            vel = vec2_add(vel, vec2_scale(lateral, lateral_speed));
            break;
        }

        case TARGET_EVASIVE: {
            /*
             * Periodically change heading by a random angle offset.
             */
            t->evasive_timer -= dt;
            if (t->evasive_timer <= 0.0) {
                /* New random evasion angle in [-60, +60] degrees */
                double max_turn = M_PI / 3.0;
                t->evasive_angle = ((double)rand() / RAND_MAX * 2.0 - 1.0) * max_turn;
                t->evasive_timer = t->evasive_period +
                                   ((double)rand() / RAND_MAX - 0.5) * 1.0;
            }
            vel = vec2_rotate(t->base_vel, t->evasive_angle);
            break;
        }
    }

    /* Clamp to max speed and apply */
    vel = vec2_clamp_mag(vel, t->body.max_speed);
    t->body.vel = vel;
    physics_step(&t->body, dt);
}

const char *target_mode_name(TargetMotionMode mode)
{
    switch (mode) {
        case TARGET_LINEAR:     return "Linear";
        case TARGET_SINUSOIDAL: return "Sinusoidal";
        case TARGET_EVASIVE:    return "Evasive";
        default:                return "Unknown";
    }
}
