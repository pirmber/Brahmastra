/*
 * ui.c — Terminal UI Implementation
 */

#include "ui.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define COL_WIDTH 72

void ui_print_banner(void)
{
    printf("\n");
    printf(UI_CYAN UI_BOLD);
    printf("  ╔══════════════════════════════════════════════════════════════════╗\n");
    printf("  ║          MISSILE GUIDANCE SIMULATION ENGINE  v1.0               ║\n");
    printf("  ║          Physics-Based Intercept Trajectory Analysis             ║\n");
    printf("  ╚══════════════════════════════════════════════════════════════════╝\n");
    printf(UI_RESET "\n");
    printf(UI_DIM "  Guidance modes : Proportional Navigation | PID\n");
    printf("  Target modes   : Linear | Sinusoidal | Evasive\n");
    printf("  Output         : CSV trajectory log + terminal stats\n" UI_RESET);
    printf("\n");
}

void ui_separator(void)
{
    printf(UI_DIM "  ──────────────────────────────────────────────────────────────────\n" UI_RESET);
}

void ui_print_progress(double elapsed, double max_duration,
                       double distance, double missile_speed)
{
    int bar_width = 30;
    double pct    = elapsed / max_duration;
    int filled    = (int)(pct * bar_width);

    printf("\r  " UI_CYAN "[");
    for (int i = 0; i < bar_width; i++)
        printf(i < filled ? "█" : "░");
    printf("]" UI_RESET
           "  t=%6.2fs  dist=%8.1fm  speed=%6.1fm/s  ",
           elapsed, distance, missile_speed);
    fflush(stdout);
}

double ui_prompt_double(const char *label, double default_val)
{
    char buf[64];
    printf("  " UI_YELLOW "%-42s" UI_RESET " [%.4g] : ", label, default_val);
    fflush(stdout);
    if (!fgets(buf, sizeof(buf), stdin) || buf[0] == '\n')
        return default_val;
    char *end;
    double v = strtod(buf, &end);
    return (end == buf) ? default_val : v;
}

int ui_prompt_int(const char *label, int default_val)
{
    char buf[64];
    printf("  " UI_YELLOW "%-42s" UI_RESET " [%d] : ", label, default_val);
    fflush(stdout);
    if (!fgets(buf, sizeof(buf), stdin) || buf[0] == '\n')
        return default_val;
    char *end;
    long v = strtol(buf, &end, 10);
    return (end == buf) ? default_val : (int)v;
}

int ui_prompt_menu(const char *label, const char **options, int count, int default_idx)
{
    printf("\n  " UI_BOLD "%s\n" UI_RESET, label);
    for (int i = 0; i < count; i++) {
        printf("    [%d] %s%s\n", i, options[i],
               (i == default_idx) ? UI_DIM " (default)" UI_RESET : "");
    }
    printf("  Choice [%d] : ", default_idx);
    fflush(stdout);

    char buf[16];
    if (!fgets(buf, sizeof(buf), stdin) || buf[0] == '\n')
        return default_idx;
    char *end;
    long v = strtol(buf, &end, 10);
    if (end == buf || v < 0 || v >= count) return default_idx;
    return (int)v;
}

void ui_configure(SimConfig *cfg)
{
    printf(UI_BOLD "\n  Configure simulation parameters\n" UI_RESET);
    printf(UI_DIM "  Press Enter to accept defaults shown in [brackets]\n\n" UI_RESET);

    /* ── Guidance mode ── */
    const char *guidance_opts[] = {
        "Proportional Navigation (PN)",
        "PID Heading Correction"
    };
    cfg->guidance_mode = (GuidanceMode)ui_prompt_menu(
        "Guidance Algorithm", guidance_opts, 2, (int)cfg->guidance_mode);

    if (cfg->guidance_mode == GUIDANCE_PN) {
        cfg->pn_nav_constant = ui_prompt_double(
            "  PN navigation constant (N, 3-5)", cfg->pn_nav_constant);
    } else {
        cfg->pid_kp = ui_prompt_double("  PID  Kp (proportional)", cfg->pid_kp);
        cfg->pid_ki = ui_prompt_double("  PID  Ki (integral)",     cfg->pid_ki);
        cfg->pid_kd = ui_prompt_double("  PID  Kd (derivative)",   cfg->pid_kd);
    }

    /* ── Target mode ── */
    const char *target_opts[] = { "Linear", "Sinusoidal", "Evasive" };
    cfg->target_mode = (TargetMotionMode)ui_prompt_menu(
        "\nTarget Motion Mode", target_opts, 3, (int)cfg->target_mode);

    /* ── Missile params ── */
    printf("\n  " UI_BOLD "Missile parameters\n" UI_RESET);
    cfg->missile_max_speed = ui_prompt_double(
        "  Max speed (m/s)", cfg->missile_max_speed);
    cfg->missile_max_accel = ui_prompt_double(
        "  Max acceleration (m/s²)", cfg->missile_max_accel);
    cfg->missile_drag = ui_prompt_double(
        "  Drag coefficient (0=off)", cfg->missile_drag);

    /* ── Target params ── */
    printf("\n  " UI_BOLD "Target parameters\n" UI_RESET);
    cfg->target_max_speed = ui_prompt_double(
        "  Target max speed (m/s)", cfg->target_max_speed);

    /* ── Simulation params ── */
    printf("\n  " UI_BOLD "Simulation parameters\n" UI_RESET);
    cfg->dt = ui_prompt_double(
        "  Timestep dt (s)", cfg->dt);
    cfg->max_duration = ui_prompt_double(
        "  Max duration (s)", cfg->max_duration);

    /* ── Randomize ── */
    const char *rand_opts[] = { "No — use configured positions", "Yes — randomize start" };
    cfg->randomize = ui_prompt_menu(
        "\nRandomize initial positions?", rand_opts, 2, cfg->randomize);
}
