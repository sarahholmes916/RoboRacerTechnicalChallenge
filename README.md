# Penn RoboRacer Technical Challenge

### Autonomous Wall Following | AutoDRIVE IROS 2026

This project implements a LiDAR-based autonomous wall-following controller for the Penn RoboRacer Technical Challenge using ROS 2 and the AutoDRIVE RoboRacer IROS 2026 practice simulator.

The objective was to complete three consecutive collision-free laps and minimize lap time through systematic controller optimization.

## Performance Results

| Metric | Result |
|---|---|
| **Fastest clean lap** | **8.589 seconds** |
| Optimized C4 average lap time | 8.746 seconds |
| C4 completion rate | 13/15 runs (86.7%) |
| Reliable baseline average | 9.572 seconds |
| Baseline completion rate | 7/7 runs (100%) |
| Improvement over automated baseline | 8.6% |

The fastest configuration, C4, achieved an **8.589-second collision-free lap**, with an average of 8.746 seconds across successful runs. C4 completed 13 of 15 validation runs, with one collision and one hairpin stall.

The recommended default configuration prioritizes reliability and completed all seven tested runs without collisions or stalls.

**Demonstration video:** 
https://www.youtube.com/watch?v=UhO5QIuMNVc

## Controller Implementation

The controller is implemented in `autodrive_devkit/autodrive_roboracer/wall_follow_node.py`.

It uses:

- **LiDAR wall estimation:** Estimates distance and orientation relative to the track boundary using side-facing and diagonal LiDAR measurements.
- **PD steering control:** Computes steering corrections from predicted wall-distance error.
- **Lookahead prediction:** Anticipates future wall distance to improve cornering.
- **Adaptive throttle:** Adjusts throttle based on steering demand and forward obstacle clearance.
- **Safety mechanisms:** Validates sensor data, monitors obstacles, and stops when unsafe conditions are detected.

The controller uses classical control methods rather than a trained neural network.

## Performance Optimization

I developed and tuned the controller through iterative simulator experiments, followed by automated hyperparameter optimization using Optuna.

| Development stage | Average lap time |
|---|---:|
| Initial wall-following implementation | 10.237 s |
| First adaptive-throttle tuning | 9.520 s |
| Automated baseline | 9.572 s |
| Optimized Trial 4 | 8.942 s |
| C3 — modified braking distance | 8.855 s |
| **C4 — optimized lookahead** | **8.746 s** |

Early experiments focused on steering gains, lookahead distance, and throttle scheduling. Subsequent optimization jointly explored throttle, braking distances, LiDAR preview parameters, and lookahead.

I implemented `optimize_controller.py` to automate simulator resets, controller parameter testing, lap-time collection, collision detection, and experiment evaluation.

Automated trials require three consecutive collision-free flying laps following a warm-up lap. Failed trials are excluded from average lap-time calculations.

The automated experiments demonstrated that reducing unnecessary braking and adjusting lookahead could improve lap times without simply increasing maximum throttle.

The most aggressive configuration was faster but less reliable, so the original stable configuration remains the default.

## Tools and Dependencies

- Python
- ROS 2 Humble
- Docker Desktop
- AutoDRIVE RoboRacer IROS 2026 Practice Simulator
- NumPy
- **Optuna** — optional, for automated hyperparameter optimization

ROS 2 and the controller's runtime dependencies are provided through the Docker environment.

To run the automated optimization experiments, install Optuna in a separate Python environment:

```powershell
python -m pip install optuna
```

Then run the optimizer from the repository root:

```powershell
python autodrive_devkit/autodrive_roboracer/optimize_controller.py --help
```

Optuna is not necessary to run the autonomous wall-following controller.

## Repository Structure

```text
RoboRacerTechnicalChallenge/
├── autodrive_devkit/
│   ├── autodrive_roboracer/
│   │   ├── wall_follow_node.py
│   │   ├── optimize_controller.py
│   │   └── ...
│   ├── test/
│   ├── setup.py
│   └── ...
├── README.md
└── WALL_FOLLOW_READ_ME.md
```

## Detailed Technical Documentation

For a complete explanation of the implementation, including:

- LiDAR geometry and PD control equations
- ROS 2 interfaces and execution instructions
- Controller parameter descriptions
- Automated optimization methodology
- Individual tuning iterations and performance improvements
- Failed experiments and hairpin reliability analysis
- Testing, verification, and reproducibility

See **[Detailed Wall-Following Documentation](autodrive_devkit/WALL_FOLLOW_README.md)**.

---

**Technical Challenge:** Penn RoboRacer

**Simulation Platform:** AutoDRIVE RoboRacer — IROS 2026 Practice Track

**Fastest Verified Collision-Free Lap:** 8.589 seconds