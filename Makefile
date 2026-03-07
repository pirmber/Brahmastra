# Makefile — Missile Guidance Simulation
# Targets: all, clean, run, run-quick, run-pid, run-evasive

CC      := gcc
CFLAGS  := -std=c99 -Wall -Wextra -Wpedantic -O2 -Iinclude -D_GNU_SOURCE
LDFLAGS := -lm

SRC_DIR := src
OBJ_DIR := obj
OUT_DIR := output
BIN     := missile_sim

SRCS := $(wildcard $(SRC_DIR)/*.c)
OBJS := $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRCS))

.PHONY: all clean run run-quick run-pid run-evasive run-sin run-rand setup

all: setup $(BIN)

$(BIN): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "  [OK] Built: $(BIN)"

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

setup:
	@mkdir -p $(OUT_DIR)

clean:
	rm -rf $(OBJ_DIR) $(BIN) $(OUT_DIR)/*.csv $(OUT_DIR)/*.png

# ── Run targets ──────────────────────────────────────────────────────────────

run: all
	./$(BIN)

run-quick: all
	./$(BIN) --quick

run-pid: all
	./$(BIN) --pid

run-pid-evasive: all
	./$(BIN) --pid --evasive

run-evasive: all
	./$(BIN) --pn --evasive

run-sin: all
	./$(BIN) --pn --sin

run-rand: all
	./$(BIN) --quick --rand

# ── Visualization ────────────────────────────────────────────────────────────

plot: $(OUT_DIR)/trajectory.csv
	python3 scripts/plot_trajectory.py

plot-static: $(OUT_DIR)/trajectory.csv
	python3 scripts/plot_trajectory.py --static

# ── Combined: simulate then immediately plot ─────────────────────────────────

demo: all
	./$(BIN) --quick
	python3 scripts/plot_trajectory.py

demo-evasive: all
	./$(BIN) --pn --evasive --quick
	python3 scripts/plot_trajectory.py

demo-pid: all
	./$(BIN) --pid --sin --quick
	python3 scripts/plot_trajectory.py
