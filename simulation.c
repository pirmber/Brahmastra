/*
 * simulation.c — Core Simulation Loop
 */

#include "simulation.h"
#include "ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#define M_PI 3.14159265358979323846

/* Write CSV header */
static void csv_write_header(FILE *f)
{
    fprintf(f,
        "time_s,"
        "missile_x,missile_y,missile_vx,missile_vy,missile_speed,"
        "target_x,target_y,target_vx,target_vy,"
        "distance_m,los_angle_deg\n");
}

/* Write one row of trajectory data */
static void csv_write_row(FILE *f, const Simulation *sim)
{
    const PhysicsBody *m = &sim->missile;
    const PhysicsBody *t = &sim->target.body;

    Vec2   rel  = vec2_sub(t->pos, m->pos);
    double dist = vec2_mag(rel);
    double los  = atan2(rel.y, rel.x) * 180.0 / M_PI;

    fprintf(f, "%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            sim->elapsed,
            m->pos.x, m->pos.y, m->vel.x, m->vel.y, m->speed,
            t->pos.x, t->pos.y, t->vel.x, t->vel.y,
            dist, los);
}

void simconfig_defaults(SimConfig *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->missile_x         = 0.0;
    cfg->missile_y         = 0.0;
    cfg->missile_vx        = 100.0;
    cfg->missile_vy        = 20.0;
    cfg->missile_max_speed = 350.0;
    cfg->missile_max_accel = 120.0;
    cfg->missile_drag      = 0.0;

    cfg->target_x          = 3000.0;
    cfg->target_y          = 1500.0;
    cfg->target_vx         = -60.0;
    cfg->target_vy         = 10.0;
    cfg->target_max_speed  = 80.0;
    cfg->target_mode       = TARGET_LINEAR;

    cfg->guidance_mode     = GUIDANCE_PN;
    cfg->pn_nav_constant   = 4.0;
    cfg->pid_kp            = 8.0;
    cfg->pid_ki            = 0.1;
    cfg->pid_kd            = 2.0;

    cfg->dt                = 0.01;
    cfg->max_duration      = SIM_MAX_DURATION;
    cfg->randomize         = 0;
}

/* Apply randomization to initial positions / velocities */
static void apply_randomization(SimConfig *cfg)
{
    srand((unsigned)time(NULL));
    double angle = ((double)rand() / RAND_MAX) * 2.0 * M_PI;
    double range = 2000.0 + ((double)rand() / RAND_MAX) * 2000.0;
    cfg->target_x = range * cos(angle);
    cfg->target_y = range * sin(angle);

    double tspeed = 40.0 + ((double)rand() / RAND_MAX) * 60.0;
    double tangle = ((double)rand() / RAND_MAX) * 2.0 * M_PI;
    cfg->target_vx = tspeed * cos(tangle);
    cfg->target_vy = tspeed * sin(tangle);
}

void simconfig_print(const SimConfig *cfg)
{
    printf("\n");
    ui_separator();
    printf(UI_BOLD UI_CYAN "  SIMULATION PARAMETERS\n" UI_RESET);
    ui_separator();
    printf(UI_BOLD "  Missile\n" UI_RESET);
    printf("    Initial position : (%.0f, %.0f) m\n",  cfg->missile_x, cfg->missile_y);
    printf("    Initial velocity : (%.1f, %.1f) m/s\n", cfg->missile_vx, cfg->missile_vy);
    printf("    Max speed        : %.1f m/s (%.1f km/h)\n",
           cfg->missile_max_speed, cfg->missile_max_speed * 3.6);
    printf("    Max acceleration : %.1f m/s²  (%.2f g)\n",
           cfg->missile_max_accel, cfg->missile_max_accel / 9.81);
    printf("    Drag coefficient : %.4f\n", cfg->missile_drag);
    printf(UI_BOLD "  Target\n" UI_RESET);
    printf("    Initial position : (%.0f, %.0f) m\n",  cfg->target_x, cfg->target_y);
    printf("    Initial velocity : (%.1f, %.1f) m/s\n", cfg->target_vx, cfg->target_vy);
    printf("    Max speed        : %.1f m/s\n",  cfg->target_max_speed);
    printf("    Motion mode      : %s\n",  target_mode_name(cfg->target_mode));
    printf(UI_BOLD "  Guidance\n" UI_RESET);
    printf("    Algorithm        : %s\n",  guidance_mode_name(cfg->guidance_mode));
    if (cfg->guidance_mode == GUIDANCE_PN)
        printf("    Nav constant (N) : %.2f\n", cfg->pn_nav_constant);
    else
        printf("    PID gains        : Kp=%.2f  Ki=%.3f  Kd=%.2f\n",
               cfg->pid_kp, cfg->pid_ki, cfg->pid_kd);
    printf(UI_BOLD "  Simulation\n" UI_RESET);
    printf("    Timestep         : %.4f s  (%.0f Hz)\n", cfg->dt, 1.0 / cfg->dt);
    printf("    Max duration     : %.1f s\n",  cfg->max_duration);
    printf("    Randomize        : %s\n",  cfg->randomize ? "Yes" : "No");
    ui_separator();
    printf("\n");
}

int sim_init(Simulation *sim, const SimConfig *cfg)
{
    memset(sim, 0, sizeof(*sim));

    SimConfig c = *cfg;
    if (c.randomize) apply_randomization(&c);

    sim->dt           = c.dt;
    sim->max_duration = c.max_duration;
    sim->min_distance = 1e18;
    sim->log_interval = 1;  /* Log every step */

    /* Initialize missile */
    physics_init(&sim->missile,
                 vec2(c.missile_x, c.missile_y),
                 vec2(c.missile_vx, c.missile_vy),
                 c.missile_max_speed,
                 c.missile_max_accel,
                 c.missile_drag, 100.0);

    /* Initialize target */
    target_init(&sim->target,
                vec2(c.target_x, c.target_y),
                vec2(c.target_vx, c.target_vy),
                c.target_mode,
                c.target_max_speed);

    /* Initialize guidance */
    guidance_init(&sim->guidance,
                  c.guidance_mode,
                  c.missile_max_accel,
                  c.pn_nav_constant,
                  c.pid_kp, c.pid_ki, c.pid_kd);

    /* Open CSV */
    sim->csv_file = fopen(SIM_CSV_PATH, "w");
    if (!sim->csv_file) {
        fprintf(stderr, "Warning: could not open %s for writing.\n", SIM_CSV_PATH);
        return -1;
    }
    csv_write_header(sim->csv_file);

    return 0;
}

void sim_run(Simulation *sim)
{
    printf(UI_BOLD UI_GREEN "  ► Running simulation...\n\n" UI_RESET);

    while (sim->elapsed <= sim->max_duration) {

        /* 1. Compute guidance command */
        Vec2 accel_cmd = guidance_compute(
            &sim->guidance,
            sim->missile.pos, sim->missile.vel,
            sim->target.body.pos, sim->target.body.vel,
            sim->dt);

        /* 2. Apply additional thrust along current heading for speed maintenance */
        Vec2 thrust_dir = vec2_normalize(sim->missile.vel);
        double speed_deficit = sim->missile.max_speed - sim->missile.speed;
        double thrust = (speed_deficit > 0.0)
                        ? fmin(speed_deficit / sim->dt, sim->missile.max_accel * 0.4)
                        : 0.0;
        Vec2 total_accel = vec2_add(accel_cmd, vec2_scale(thrust_dir, thrust));

        /* 3. Apply to missile and step physics */
        physics_apply_accel(&sim->missile, total_accel);
        physics_step(&sim->missile, sim->dt);

        /* 4. Step target */
        target_step(&sim->target, sim->dt);

        /* 5. Collision detection */
        double dist = vec2_dist(sim->missile.pos, sim->target.body.pos);

        if (dist < sim->min_distance) {
            sim->min_distance  = dist;
            sim->min_dist_time = sim->elapsed;
        }

        if (dist <= SIM_INTERCEPT_RADIUS) {
            sim->intercept     = 1;
            sim->intercept_time = sim->elapsed;
            sim->intercept_pos  = sim->missile.pos;
        }

        /* 6. Log data */
        if (sim->step_count % sim->log_interval == 0 && sim->csv_file)
            csv_write_row(sim->csv_file, sim);

        /* 7. Terminal progress (every 50 steps) */
        if (sim->step_count % 50 == 0)
            ui_print_progress(sim->elapsed, sim->max_duration, dist, sim->missile.speed);

        sim->elapsed    += sim->dt;
        sim->step_count++;

        if (sim->intercept) break;
    }

    printf("\n\n");
}

void sim_cleanup(Simulation *sim)
{
    if (sim->csv_file) {
        fclose(sim->csv_file);
        sim->csv_file = NULL;
    }
}

void sim_print_results(const Simulation *sim)
{
    ui_separator();
    printf(UI_BOLD UI_CYAN "  SIMULATION RESULTS\n" UI_RESET);
    ui_separator();

    if (sim->intercept) {
        printf(UI_BOLD UI_GREEN "  STATUS      : INTERCEPT\n" UI_RESET);
        printf("  Time        : %.3f s\n", sim->intercept_time);
        printf("  Position    : (%.1f, %.1f) m\n",
               sim->intercept_pos.x, sim->intercept_pos.y);
    } else {
        printf(UI_BOLD UI_RED "  STATUS      : MISS (timeout)\n" UI_RESET);
    }

    printf("  Min Dist    : %.2f m  @ t=%.3f s\n",
           sim->min_distance, sim->min_dist_time);
    printf("  Kill Radius : %.1f m\n", SIM_INTERCEPT_RADIUS);
    printf("  Steps Run   : %d  (dt=%.4f s)\n", sim->step_count, sim->dt);

    if (sim->csv_file || 1)
        printf("  CSV Output  : %s\n", SIM_CSV_PATH);

    ui_separator();
    printf("\n");
}
