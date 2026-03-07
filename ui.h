/*
 * ui.h — Terminal UI Utilities
 *
 * ANSI color output, progress display, and parameter prompting.
 */

#ifndef UI_H
#define UI_H

#include "simulation.h"

/* ANSI color codes */
#define UI_RESET   "\033[0m"
#define UI_BOLD    "\033[1m"
#define UI_DIM     "\033[2m"
#define UI_RED     "\033[31m"
#define UI_GREEN   "\033[32m"
#define UI_YELLOW  "\033[33m"
#define UI_CYAN    "\033[36m"
#define UI_WHITE   "\033[37m"
#define UI_BLUE    "\033[34m"
#define UI_MAGENTA "\033[35m"

/* Print the simulation banner */
void ui_print_banner(void);

/* Print simulation progress line (overwrite in-place) */
void ui_print_progress(double elapsed, double max_duration,
                       double distance, double missile_speed);

/* Interactive parameter configuration */
void ui_configure(SimConfig *cfg);

/* Print separator line */
void ui_separator(void);

/* Prompt for a double with default */
double ui_prompt_double(const char *label, double default_val);

/* Prompt for an int with default */
int ui_prompt_int(const char *label, int default_val);

/* Prompt for a menu selection */
int ui_prompt_menu(const char *label, const char **options, int count, int default_idx);

#endif /* UI_H */
