# Missile Guidance Simulation Engine

A physics-based 2D missile intercept simulation implementing two
classical guidance laws: **Proportional Navigation (PN)** and **PID
heading correction**. Written in C99 with a Python/matplotlib
visualization layer.

---

## Project Structure

```
missile_sim/
├── include/
│   ├── vec2.h          — 2D vector math (header-only)
│   ├── physics.h       — Rigid body physics
│   ├── target.h        — Moving target simulation
│   ├── guidance.h      — PN and PID guidance algorithms
│   ├── simulation.h    — Top-level simulation state
│   └── ui.h            — Terminal UI utilities
├── src/
│   ├── main.c          — Entry point, CLI parsing
│   ├── physics.c       — Euler integration, drag
│   ├── target.c        — Linear / sinusoidal / evasive motion
│   ├── guidance.c      — Proportional Navigation + PID
│   ├── simulation.c    — Simulation loop, CSV logging, metrics
│   └── ui.c            — ANSI terminal UI, parameter prompts
├── scripts/
│   └── plot_trajectory.py — matplotlib animator + static plotter
├── output/             — Generated CSV and PNG files
├── Makefile
└── README.md
```

---

## Build

**Requirements:** GCC (C99), GNU Make, math library (`-lm`)

```bash
make          # build
make run      # build + interactive launch
make run-quick  # build + launch with defaults
```

---

## Run Options

```bash
./missile_sim              # interactive parameter configuration
./missile_sim --quick      # run with defaults immediately
./missile_sim --pn         # Proportional Navigation, no prompts
./missile_sim --pid        # PID guidance, no prompts
./missile_sim --sin        # sinusoidal target
./missile_sim --evasive    # evasive target
./missile_sim --rand       # randomize start positions
./missile_sim --help       # show flags
```

Flags may be combined:

```bash
./missile_sim --pid --evasive --rand
```

---

## Visualization

After a simulation run:

```bash
make plot          # animated + static PNG
make plot-static   # static PNG only

# or directly:
python3 scripts/plot_trajectory.py
python3 scripts/plot_trajectory.py --static
python3 scripts/plot_trajectory.py --csv output/trajectory.csv
```

**Python requirements:**

```
pip install numpy pandas matplotlib
```

---

## Make Shortcuts

| Command              | Description                              |
|----------------------|------------------------------------------|
| `make demo`          | Default sim + animate result             |
| `make demo-evasive`  | PN vs evasive target + plot              |
| `make demo-pid`      | PID vs sinusoidal target + plot          |
| `make run-rand`      | Randomized positions, defaults           |
| `make clean`         | Remove build artifacts and output files  |

---

## Guidance Algorithms

### Proportional Navigation (PN)

```
a_cmd = N · V_c · λ̇
```

- `N`   — navigation constant (3–5)
- `V_c` — closing velocity (rate of range decrease)
- `λ̇`  — LOS angle rate of change

The missile accelerates perpendicular to the line of sight to null
its rotation. Highly effective against non-maneuvering targets; good
against weaving ones with higher N.

### PID Guidance

```
error       = wrap(LOS_angle - missile_heading)
a_lateral   = Kp·e + Ki·∫e·dt + Kd·(de/dt)
```

Computes angular heading error and drives it to zero with a PID
controller. More tunable for different target behaviors; can
overshoot at high speeds if gains are too aggressive.

---

## Output Format

`output/trajectory.csv` columns:

| Column           | Units | Description                  |
|------------------|-------|------------------------------|
| `time_s`         | s     | Simulation time              |
| `missile_x/y`    | m     | Missile world position       |
| `missile_vx/vy`  | m/s   | Missile velocity components  |
| `missile_speed`  | m/s   | Missile speed magnitude      |
| `target_x/y`     | m     | Target world position        |
| `target_vx/vy`   | m/s   | Target velocity components   |
| `distance_m`     | m     | Miss distance                |
| `los_angle_deg`  | deg   | Line-of-sight angle          |

---

## Physics Notes

- **Integration**: Semi-implicit Euler (v += a·dt, x += v·dt)
- **Drag**: Quadratic aerodynamic drag, `F = -Cd · |v| · v / m`
- **Speed limit**: Velocity clamped to `max_speed` after integration
- **Accel limit**: Command clamped to `max_accel` before application
- **Intercept**: Triggered when missile–target distance ≤ 5.0 m

---

## License

MIT — For educational and research use only. This is a mathematical
and control systems simulation. No real-world weapon construction
details are present or implied.
