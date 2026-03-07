/*
 * simulation.h — Top-level Simulation Configuration and State
 */

#ifndef SIMULATION_H
#define SIMULATION_H

#include "physics.h"
#include "target.h"
#include "guidance.h"
#include <stdio.h>

#define SIM_MAX_DURATION     120.0   /* Max simulation time (s)       */
#define SIM_INTERCEPT_RADIUS   5.0   /* Proximity kill radius (m)     */
#define SIM_CSV_PATH   "output/trajectory.csv"

typedef struct {
    /* Time */
    double   dt;               /* Timestep (s)                     */
    double   elapsed;          /* Elapsed simulation time (s)      */
    double   max_duration;     /* Max allowed run time (s)         */

    /* Entities */
    PhysicsBody    missile;
    Target         target;
    GuidanceSystem guidance;

    /* Metrics */
    double   min_distance;     /* Closest approach distance (m)    */
    double   min_dist_time;    /* Time of closest approach (s)     */
    int      intercept;        /* 1 = intercept occurred           */
    double   intercept_time;   /* Time of intercept (s)            */
    Vec2     intercept_pos;    /* Position of intercept            */

    /* I/O */
    FILE    *csv_file;         /* Trajectory log                   */
    int      log_interval;     /* Log every N steps                */
    int      step_count;       /* Current step number              */
} Simulation;

typedef struct {
    /* Missile */
    double missile_x, missile_y;
    double missile_vx, missile_vy;
    double missile_max_speed;
    double missile_max_accel;
    double missile_drag;

    /* Target */
    double target_x, target_y;
    double target_vx, target_vy;
    double target_max_speed;
    TargetMotionMode target_mode;

    /* Guidance */
    GuidanceMode guidance_mode;
    double pn_nav_constant;
    double pid_kp, pid_ki, pid_kd;

    /* Simulation */
    double dt;
    double max_duration;
    int    randomize;
} SimConfig;

/* Initialize simulation from config */
int  sim_init(Simulation *sim, const SimConfig *cfg);

/* Run the full simulation loop */
void sim_run(Simulation *sim);

/* Cleanup (close CSV file, etc.) */
void sim_cleanup(Simulation *sim);

/* Print final stats to terminal */
void sim_print_results(const Simulation *sim);

/* Apply default config values */
void simconfig_defaults(SimConfig *cfg);

/* Print config summary */
void simconfig_print(const SimConfig *cfg);

#endif /* SIMULATION_H */
