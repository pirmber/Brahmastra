/*
 * main.c — Missile Guidance Simulation Entry Point
 *
 * Usage:
 *   ./missile_sim          — interactive configuration
 *   ./missile_sim --quick  — run immediately with defaults
 *   ./missile_sim --pn     — use Proportional Navigation, no prompts
 *   ./missile_sim --pid    — use PID guidance, no prompts
 *   ./missile_sim --rand   — randomize initial positions
 *   ./missile_sim --evasive  — evasive target
 *   ./missile_sim --sin      — sinusoidal target
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "simulation.h"
#include "ui.h"

int main(int argc, char *argv[])
{
    SimConfig cfg;
    simconfig_defaults(&cfg);

    ui_print_banner();

    /* Parse command line flags */
    int quick    = 0;
    int interact = 1;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--quick")   == 0) { quick = 1; interact = 0; }
        if (strcmp(argv[i], "--pn")      == 0) { cfg.guidance_mode = GUIDANCE_PN;  interact = 0; }
        if (strcmp(argv[i], "--pid")     == 0) { cfg.guidance_mode = GUIDANCE_PID; interact = 0; }
        if (strcmp(argv[i], "--rand")    == 0) { cfg.randomize = 1; }
        if (strcmp(argv[i], "--evasive") == 0) { cfg.target_mode = TARGET_EVASIVE; }
        if (strcmp(argv[i], "--sin")     == 0) { cfg.target_mode = TARGET_SINUSOIDAL; }
        if (strcmp(argv[i], "--help")    == 0) {
            printf("  Flags: --quick --pn --pid --rand --evasive --sin\n\n");
            return 0;
        }
    }

    /* Interactive configuration unless a flag skipped it */
    if (interact && !quick) {
        ui_configure(&cfg);
    }

    /* Show final parameters */
    simconfig_print(&cfg);

    /* Initialize, run, display results */
    Simulation sim;
    if (sim_init(&sim, &cfg) < 0) {
        fprintf(stderr, "  Warning: CSV logging disabled (could not open output file).\n");
        fprintf(stderr, "  Ensure the 'output/' directory exists.\n\n");
    }

    sim_run(&sim);
    sim_print_results(&sim);
    sim_cleanup(&sim);

    printf(UI_DIM "  Trajectory data written to: " UI_RESET
           UI_CYAN SIM_CSV_PATH UI_RESET "\n");
    printf(UI_DIM "  Visualize with: " UI_RESET
           UI_CYAN "python3 scripts/plot_trajectory.py\n\n" UI_RESET);

    return sim.intercept ? 0 : 1;
}
