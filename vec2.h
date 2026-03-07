/*
 * vec2.h — 2D Vector Math Library
 *
 * Provides fundamental vector operations used throughout the simulation.
 * All functions operate on Vec2 structs passed by value for clarity.
 */

#ifndef VEC2_H
#define VEC2_H

#include <math.h>

typedef struct {
    double x;
    double y;
} Vec2;

/* Construction */
static inline Vec2 vec2(double x, double y)        { return (Vec2){x, y}; }
static inline Vec2 vec2_zero(void)                  { return (Vec2){0.0, 0.0}; }

/* Arithmetic */
static inline Vec2 vec2_add(Vec2 a, Vec2 b)         { return (Vec2){a.x+b.x, a.y+b.y}; }
static inline Vec2 vec2_sub(Vec2 a, Vec2 b)         { return (Vec2){a.x-b.x, a.y-b.y}; }
static inline Vec2 vec2_scale(Vec2 v, double s)     { return (Vec2){v.x*s, v.y*s}; }

/* Geometry */
static inline double vec2_dot(Vec2 a, Vec2 b)       { return a.x*b.x + a.y*b.y; }
static inline double vec2_cross(Vec2 a, Vec2 b)     { return a.x*b.y - a.y*b.x; }
static inline double vec2_mag(Vec2 v)               { return sqrt(v.x*v.x + v.y*v.y); }
static inline double vec2_mag_sq(Vec2 v)            { return v.x*v.x + v.y*v.y; }

/* Normalize — returns zero vector if magnitude is near zero */
static inline Vec2 vec2_normalize(Vec2 v) {
    double m = vec2_mag(v);
    if (m < 1e-10) return vec2_zero();
    return vec2_scale(v, 1.0 / m);
}

/* Angle of vector from positive X axis, radians */
static inline double vec2_angle(Vec2 v) { return atan2(v.y, v.x); }

/* Clamp magnitude to max_mag */
static inline Vec2 vec2_clamp_mag(Vec2 v, double max_mag) {
    double m = vec2_mag(v);
    if (m > max_mag && m > 1e-10)
        return vec2_scale(v, max_mag / m);
    return v;
}

/* Distance between two points */
static inline double vec2_dist(Vec2 a, Vec2 b) { return vec2_mag(vec2_sub(b, a)); }

/* Rotate vector by angle (radians) */
static inline Vec2 vec2_rotate(Vec2 v, double angle) {
    double c = cos(angle), s = sin(angle);
    return (Vec2){ v.x*c - v.y*s, v.x*s + v.y*c };
}

#endif /* VEC2_H */
