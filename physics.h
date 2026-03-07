/*
 * physics.h — Rigid Body Physics for 2D Simulation
 *
 * Defines a generic physics body with position, velocity, heading,
 * drag, and speed/acceleration limits.
 */

#ifndef PHYSICS_H
#define PHYSICS_H

#include "vec2.h"

typedef struct {
    Vec2   pos;           /* World position (m)            */
    Vec2   vel;           /* Velocity (m/s)                */
    Vec2   accel;         /* Applied acceleration (m/s²)   */
    double heading;       /* Current heading angle (rad)   */
    double speed;         /* Current speed magnitude (m/s) */
    double max_speed;     /* Speed limit (m/s)             */
    double max_accel;     /* Acceleration limit (m/s²)     */
    double drag_coeff;    /* Drag coefficient (0 = off)    */
    double mass;          /* Mass (kg), used for drag      */
} PhysicsBody;

/* Initialize body with position, velocity, and limits */
void physics_init(PhysicsBody *body,
                  Vec2 pos, Vec2 vel,
                  double max_speed, double max_accel,
                  double drag_coeff, double mass);

/* Integrate one timestep (Euler integration, dt in seconds) */
void physics_step(PhysicsBody *body, double dt);

/* Apply acceleration vector, clamped to body->max_accel */
void physics_apply_accel(PhysicsBody *body, Vec2 accel);

#endif /* PHYSICS_H */
