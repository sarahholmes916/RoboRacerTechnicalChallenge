# Penn RoboRacer wall follower

LiDAR wall-following controller for the IROS 2026 AutoDRIVE practice track, with a
reliable default configuration and an automated simulator-in-the-loop optimizer. All
results below are simulator measurements; they are not a guarantee for other simulator
conditions. Repeat the clean-lap check for your final recorded run.

## Results at a glance

| | Result |
| --- | --- |
| **Recommended default** (reliable) | **9.572 s** mean over 3 flying laps, fastest clean lap **9.466 s**, **7 of 7** runs clean, zero collisions |
| **Fastest clean lap** (experimental C4) | **8.589 s**; C4 mean **8.746 s**, but only **13 of 15** runs completed |
| Unlisted YouTube video | **[YOUR VIDEO URL - add before submitting]** |

Example of three consecutive collision-free laps with the recommended default
(2026-10-18, automated run): **9.634 s, 9.641 s, 9.536 s** (mean 9.604 s).

The recommended default is the verified, reliable controller. The faster configurations
(C3, C4) are experimental and not fully validated; see
[Controller Development and Performance Optimization](#controller-development-and-performance-optimization).

## Controller

`wall_follow_node.py` contains a standard-library-only controller and its ROS 2
adapter. It follows the right wall by default. Like the
[F1TENTH wall-following lab](https://github.com/f1tenth/f1tenth_lab3_template), it
uses side and diagonal LiDAR measurements, estimates wall angle and perpendicular
distance, predicts distance at a lookahead point, and applies PID steering.
The default `ki=0` makes it a PD controller.

With side range `b`, diagonal range `a`, and separation `theta`:

```text
alpha = atan2(a*cos(theta) - b, a*sin(theta))
distance = b*cos(alpha)
predicted_distance = distance + lookahead_distance*sin(alpha)
error = side*(predicted_distance - desired_wall_distance)
side = -1 for right, +1 for left
```

The implementation uses Cartesian endpoints with the actual sampled angles,
which generalizes these formulas to non-exact beam locations. A close right wall
produces positive (leftward) steering; a close left wall produces negative
(rightward) steering. A small median window suppresses isolated side outliers.
All indexing uses `angle_min`, `angle_increment`, and actual range count.

Throttle decreases with steering magnitude and front proximity. It ramps upward
gradually but decreases immediately. The front check uses the **minimum** finite
return in a +/-20 degree cone, with valid coverage required in each third of that
cone. Positive infinity means no return within range and counts as front
clearance up to `range_max`; negative infinity forces a stop, following
[ROS REP-117](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0117.rst).
Safety can reduce throttle below `min_throttle`, all the way to zero.
If the preferred wall is unavailable, two valid measurements of the opposite
wall can provide a fallback at no more than `min_throttle`. It uses the same
desired distance and resets PID history on every reference-wall change. Disable
this with `allow_opposite_wall_fallback:=false` for strict single-side following.
If neither wall is measurable, or scans are malformed, stale, or lack front
coverage, both commands stop and PID state resets. Valid scans allow recovery.

The derivative is filtered and bounded; integral accumulation is bounded and
suppressed during saturation. A steady-clock watchdog stops after 0.3 seconds
without scans, including when simulation time is paused. Nonzero scan timestamps
must be fresh and increasing; entirely unstamped streams use receipt time.
Diagnostics report commands, estimated distance, predicted distance, and front
clearance at most once per `log_interval` (2 seconds by default).

## ROS interfaces

| Direction | Topic | Type |
| --- | --- | --- |
| Subscribe | `/autodrive/roboracer_1/lidar` | `sensor_msgs/msg/LaserScan` |
| Publish | `/autodrive/roboracer_1/throttle_command` | `std_msgs/msg/Float32` |
| Publish | `/autodrive/roboracer_1/steering_command` | `std_msgs/msg/Float32` |

Commands are normalized to [-1, 1], not m/s or radians. This controller uses
nonnegative throttle only. Positive throttle is forward; positive steering is
left. Commands use reliable QoS to match the bridge; the scan subscription accepts
best-effort and reliable publishers. No changes to the bridge are needed.

## Build and run in the existing container

The inspected `penn-challenge` container bind-mounts this repository's
`autodrive_devkit` at `/home/autodrive_devkit/src/autodrive_devkit`. No Docker image
rebuild or file copy is needed. From host PowerShell, rebuild after edits:

```powershell
docker exec penn-challenge bash -lc 'source /opt/ros/humble/setup.bash && cd /home/autodrive_devkit && colcon build --packages-select autodrive_roboracer'
```

Stop any existing wall follower or keyboard teleop with Ctrl+C in its terminal.
Keep the already-running bridge running; it is the only existing launch-file
node needed here. Start the controller in a separate terminal:

```powershell
docker exec -it penn-challenge bash -lc 'source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash && exec ros2 run autodrive_roboracer wall_follow_node'
```

Select Autonomous mode in the simulator and connect it to the bridge. The node
waits at zero throttle until usable scans arrive. To restart after edits or a
parameter change: Ctrl+C, rebuild, then run again. Do not run two command sources
simultaneously. Ctrl+C/SIGTERM publishes zeros before ROS shutdown.

Example tuning run (parameters are validated and **startup-only**; restart to
change them, rather than using `ros2 param set`):

```powershell
docker exec -it penn-challenge bash -lc 'source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash && exec ros2 run autodrive_roboracer wall_follow_node --ros-args -p wall_side:=right -p desired_wall_distance:=0.65 -p lookahead_distance:=0.8 -p kp:=1.2 -p ki:=0.0 -p kd:=0.08 -p min_throttle:=0.12 -p max_throttle:=0.26'
```

## Tuning and clean-lap procedure

| Parameter | Default | Meaning |
| --- | --- | --- |
| `wall_side` | `right` | `right` or `left` |
| `allow_opposite_wall_fallback` | `true` | Follow a measurable opposite wall at reduced throttle if the preferred wall disappears |
| `desired_wall_distance` | 0.65 m | LiDAR-to-wall target, allowing for vehicle width |
| `lookahead_distance` | 0.8 m | Predicted travel distance |
| `kp`, `ki`, `kd` | 1.2, 0.0, 0.08 | Normalized steering gains |
| `min_throttle`, `max_throttle` | 0.12, 0.26 | Corner/straight throttle targets |
| `corner_slowdown_threshold` | 0.35 | Absolute normalized steering reaching corner throttle |
| `front_obstacle_safety_distance` | 0.5 m | Full-stop distance from LiDAR |
| `front_slowdown_distance` | 1.7 m | Start reducing throttle |
| `front_sector_half_angle_deg` | 20 degrees | Front cone half-width |
| `wall_ray_separation_deg` | 45 degrees | Separation from the +/-90 degree side ray |
| `sample_half_width_deg` | 2 degrees | Side measurement filter half-width |
| `max_wall_range` | 9.5 m | Reject more distant returns as unavailable walls |
| `min_valid_fraction` | 0.6 | Minimum usable fraction in each sampled sector |
| `scan_timeout` | 0.3 s | Receipt/header freshness bound |
| `derivative_filter_tau`, `integral_limit` | 0.1 s, 0.5 | Derivative smoothing and integral bound |
| `throttle_rise_rate`, `log_interval` | 0.5/s, 2 s | Acceleration ramp and diagnostics interval |
| `optimization_enabled` | `true` (ROS node default; `false` in the unit-tested config class) | Enables adaptive corner throttle and forward-LiDAR preview braking |
| `adaptive_corner_throttle` | 0.14 (ROS node default; 0.12 in the config class) | Throttle at full corner demand when optimization is enabled |
| `preview_slowdown_distance` | 2.5 m | Forward preview starts capping throttle before a turn |
| `preview_path_half_width` | 0.3 m | Half-width of the forward preview corridor |
| `preview_half_angle_deg` | 40 degrees | Half-width of the preview fan |
| `dynamic_lookahead_enabled`, `steering_smoothing_tau` | `false`, 0 | Optional experiments, both rejected (see below) |

- **Crashes in sharp turns:** first lower maximum and minimum throttle. Increase
  lookahead modestly (e.g. 0.8 to 0.9 m) if steering starts late; lower the corner
  threshold to slow sooner. Increase front slowdown distance for earlier braking.
  If it cuts the inside wall, increase desired clearance slightly. If it stops
  nose-first, improve early steering at low throttle before changing stop margins.
- **Oscillates:** reduce Kp in 10-20% steps, keep Ki at zero, reduce throttle, and
  try a small Kd increase. If steering is noisy, increase derivative filter tau
  or the side sample window. Excessive Kd can also cause oscillation.
- **Too slow:** only after three consecutive clean laps, increase maximum throttle
  by 0.01 per trial. Increase minimum throttle separately if corners remain clean.
  If braking activates constantly, inspect reported front clearance and track
  width before adjusting the front cone or slowdown distance. Throttle is not speed.
- **Loses the wall:** inspect scan coverage and reported `wall unavailable` stops.
  Try the other wall side; adjust ray separation or increase max wall range only
  if the actual wall is farther away. Sparse isolated dropouts can benefit from a
  wider sample window. The opposite-wall fallback helps at corner openings;
  losing both walls intentionally stops. No blind wall search is attempted.

Record three **full** consecutive laps after a start-line crossing, with no
increase in `/autodrive/roboracer_1/collision_count`. Monitor `/lap_count`,
`/last_lap_time`, and `/best_lap_time` under that same topic prefix. The simulator's
best time alone does not prove that lap was clean. Keep each run's parameter
values, collision count, and lap times; change one parameter at a time, and rerun
the three-lap check after every speed increase. Preserve the fastest clean setup.

For example, open another container shell to inspect or record a run:

```powershell
docker exec -it penn-challenge bash
```

```bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
ros2 topic hz /autodrive/roboracer_1/lidar
# Ctrl+C, then record to a new bag directory; Ctrl+C finishes recording.
ros2 bag record -o penn_run_01 /autodrive/roboracer_1/lidar /autodrive/roboracer_1/steering_command /autodrive/roboracer_1/throttle_command /autodrive/roboracer_1/lap_count /autodrive/roboracer_1/last_lap_time /autodrive/roboracer_1/best_lap_time /autodrive/roboracer_1/collision_count
```

## Tests and verification

Host tests need only Python; the ROS test skips unless explicitly enabled:

```powershell
python -B -m unittest discover -s autodrive_devkit/test -p 'test_wall_follow*.py' -v
```

After building, run the real ROS adapter test on **unused test domain 87**. The
inspected bridge uses domain 0. Never run synthetic publishers on its live domain.

```powershell
docker exec -e ROS_DOMAIN_ID=87 -e ROS_LOCALHOST_ONLY=1 -e WALL_FOLLOW_ROS_TESTS=1 -e PYTHONDONTWRITEBYTECODE=1 penn-challenge bash -lc 'source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash && cd /home/autodrive_devkit/src/autodrive_devkit && python3 -B -m unittest discover -s test -p "test_wall_follow*.py" -v'
```

Implemented verification: Python syntax parsing; 26 synthetic tests on the Windows
host; successful `colcon build` in `penn-challenge`; all 27 tests passing under
ROS 2 Humble in Docker, including installed-executable DDS communication, steering
sign, invalid scan stops, watchdog with frozen ROS time, recovery, and SIGTERM
shutdown stops; console entry point discovery; focused flake8; `git diff --check`;
live LiDAR inspection and bounded simulator driving trials. The simulator check
counts four start-line crossings, excludes the initial partial lap, and requires
the collision counter to remain unchanged throughout the following three laps.

### Early controller testing (before optimization)

These were the first simulator measurements, taken with the original controller before
any of the optimization work described below. Times are reported simulator seconds, not
host wall time.

| Maximum throttle | Result |
| --- | --- |
| 0.25 | Three consecutive full clean laps; observed lap times included 10.2965 s and 10.2215 s |
| 0.30 | Stopped before completing a lap; zero reported collisions; rejected |
| **0.26 (default)** | **10.1505 s, 10.1615 s, 10.1465 s**, three consecutive full laps; collision count stayed **0** |

All other parameters were unchanged between these comparisons. The first start-line
crossing was excluded as a possible partial lap, and the controller was stopped after the
fourth crossing. This was the fastest setup in that early comparison; the later
optimization (below) is faster. The ~10.15 s figures are not directly comparable with the
~10.237 s baseline used in the optimization history, which comes from a separate set of runs.

## Limitations and optional ML

Two rays can hit different walls at corners or openings. Opposite-wall fallback
uses the same clearance target and can shift the path across wider corridors.
This is a reactive wall follower, with no map, gap planner, speed feedback, or collision prediction along
the full turning trajectory. Front distance is measured from LiDAR, not the bumper;
allow for vehicle overhang, latency, and braking distance when tuning. Zero throttle
uses the bridge's stop convention but does not prove instantaneous physical braking.
NaN and out-of-range readings are unknown, not free space. Positive infinity and
finite max-range returns count as front clearance but never as a wall; negative
infinity is too close to measure and stops the car. Conservative stops
may require manual repositioning. Both nodes must use the same clock convention;
after a backward ROS-time reset, restart the follower. The unchanged bridge has no
command watchdog: SIGKILL, a dead controller, or a lost connection can leave its
last command latched. Normal Ctrl+C/SIGTERM and live-node stale-scan stops are tested.

ML is deferred until the classical controller completes clean simulator runs.
The bag above preserves timestamps for future scan/action alignment. A later,
separate training tool should pair each observation with its ensuing controller
action, discard unsafe runs, downsample/normalize ranges, optionally include the
previous action, and train a small PyTorch MLP for normalized steering. Split by
**complete driving run**, never random adjacent frames. Validate closed-loop in
simulation and retain classical safety/fallback; offline loss is not a deployment
criterion. PyTorch is not a dependency of this controller.

## Controller Development and Performance Optimization

All lap times are simulator seconds from AutoDRIVE lap telemetry. "Clean" means zero
collisions and no safety stalls. Automated trials use one warm-up (standing-start) lap
followed by three flying laps. The earliest results (10.237 s and 9.520 s) were manual
runs, so percentages that compare them with automated results are approximate.

### 1. Initial implementation (about 10.237 s)

A geometric LiDAR wall follower in the style of the F1TENTH lab: two right-side rays (90
and 45 degrees) give wall angle and distance, distance is projected to a lookahead point
(0.8 m), and a PD controller (kp 1.2, kd 0.08, target wall distance 0.65 m) outputs
steering. Throttle falls with steering demand and front clearance, and the minimum return in
a +/-20 degree cone triggers an emergency stop at 0.5 m. Stale or invalid scans stop the
car. Average lap: about 10.237 s, three clean laps.

### 2. First tuning (about 9.52 s)

An adaptive corner throttle (0.14) plus forward-LiDAR preview braking raised straight-line
speed while slowing for corners: about 9.52 s average, clean. Corner throttle 0.15 and
maximum throttle 0.27 both failed with safety stops, which showed that throttle, braking
and steering interact and motivated joint rather than one-at-a-time tuning.

### 3. Rejected experiments

Dynamic lookahead (9.639 s), light steering smoothing (9.659 s) and a shorter preview
slowdown distance (9.533 s, within noise) did not beat the 9.520-9.543 s reference and
were reverted. The verified configuration remains the ROS node default.

### 4. Automated optimization

`autodrive_devkit/autodrive_roboracer/optimize_controller.py` runs on the host and drives
the Docker container. Each trial does a preflight check (telemetry present, no competing
publishers), resets the simulator, starts a fresh `wall_follow_node` with `-p` overrides,
measures one warm-up lap and three flying laps, and shuts the controller down cleanly. A
configuration is valid only if all three flying laps are collision-free with no stall.

The search uses Optuna TPE over throttle, braking, preview and lookahead parameters. Laps,
collisions, stalls, 10 Hz trajectories and failure diagnostics are saved to
`autodrive_devkit/data/optimization/` (git-ignored).

A pilot accidentally ran two optimizer processes at once, which contaminated several
trials. The optimizer now has a single-instance lock, refuses to start if any other
`wall_follow_node` exists, waits for each controller to exit before the next trial, and
cleans up on Ctrl+C and timeouts.

### 5. Trial 4 and the intermittent stall

Trial 4 parameters: `max_throttle` 0.2645, `adaptive_corner_throttle` 0.1218,
`corner_slowdown_threshold` 0.3691, `preview_slowdown_distance` 1.7398,
`front_slowdown_distance` 1.2879, `preview_path_half_width` 0.2956,
`throttle_rise_rate` 0.3154, `lookahead_distance` 0.9471, `kp` 1.2, `kd` 0.08.

It averaged 8.942 s over its valid runs, but **only 7 of 9 runs completed**. Trajectory
logs show the stalls at one hairpin (about x -1.5, y -7.6): the right wall drops out of view
mid-turn, steering relaxes, and the front-clearance factor drives throttle to about 0 at
0.501 m, just outside the 0.5 m emergency threshold. The car then sits motionless with no
collision and no counted safety stop; the controller has no reverse recovery. Successful
Trial 4 passes crept through that hairpin at only about 0.5 speed.

### 6. Reliability experiments

Each configuration below changes one parameter of Trial 4.

| Config | Change | Runs | Result |
| --- | --- | --- | --- |
| C1 | `adaptive_corner_throttle` 0.14 | 1 | Collision; rejected |
| C2 | `front_slowdown_distance` 1.0 | 1 | Safety stall; rejected |
| **C3** | `front_slowdown_distance` **1.5** | 4 clean of 5 | 8.855 s mean, fastest 8.766 s; hairpin minimum speed about 0.9 (baseline about 0.8). The 5th run was invalidated by a harness bug (publisher-discovery race, since fixed), not by driving, but only 4 clean runs exist: **not validated** |
| **C4** | `lookahead_distance` **0.8** | **13 clean of 15** | **8.746 s mean, fastest clean lap 8.589 s**; failures: 1 hairpin deadlock, 1 collision. **Faster, but not reliable enough to be the default** |

> **C4 summary:** the fastest clean lap observed was **8.589 s**, and C4 averaged
> **8.746 s**, but only **13 of 15** runs completed (87%), so it is documented as
> experimental rather than the default.

Not done for lack of time: the combined C3+C4 configuration, and a controller-side
recovery (for example reverse, or a throttle floor near walls).

### 7. Performance summary

| Stage | Main changes | Mean clean lap (s) | Fastest clean lap (s) | Reliability | vs. baseline |
| --- | --- | --- | --- | --- | --- |
| Original controller (manual) | PD wall following | 10.237 | n/a | 3 clean laps | reference |
| First tuning (manual) | Adaptive corner throttle 0.14 | 9.520 | n/a | 3 clean laps | -7.0% vs 10.237 |
| **Recommended default** (automated protocol) | Same settings | **9.572** | 9.466 | **7/7 runs** | -6.5% vs 10.237 |
| Dynamic lookahead / smoothing | Rejected | 9.639 / 9.659 | n/a | Clean but slower | Slower |
| Trial 4 | Joint TPE-tuned parameters | 8.942 | 8.634 | 7/9 runs | -6.6% vs 9.572 |
| C3 (experimental) | Trial 4 + `front_slowdown_distance` 1.5 | 8.855 | 8.766 | 4/5 (1 harness error) | -7.5% vs 9.572 |
| **C4 (experimental)** | Trial 4 + `lookahead_distance` 0.8 | **8.746** | **8.589** | 13/15 runs | -8.6% vs 9.572 |

Percentages compare means within the same automated protocol (9.572 s baseline); the manual
10.237 s and 9.520 s figures were measured separately.

### 8. Reproducing the results

Start the simulator and bridge as described in the repository README ("Competition
Execution"), then run the controller in the devkit container (`penn-challenge`). This runs
the recommended default:

```powershell
docker exec -it penn-challenge bash -lc 'source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash && exec ros2 run autodrive_roboracer wall_follow_node'
```

Experimental C4 (fastest, 13 of 15 runs completed):

```powershell
docker exec -it penn-challenge bash -lc 'source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash && exec ros2 run autodrive_roboracer wall_follow_node --ros-args -p optimization_enabled:=true -p max_throttle:=0.2645 -p adaptive_corner_throttle:=0.1218 -p corner_slowdown_threshold:=0.3691 -p preview_slowdown_distance:=1.7398 -p front_slowdown_distance:=1.2879 -p preview_path_half_width:=0.2956 -p throttle_rise_rate:=0.3154 -p lookahead_distance:=0.8'
```

Experimental C3 (4 clean runs, not validated): use the same command with
`front_slowdown_distance:=1.5` and `lookahead_distance:=0.9471`.

To repeat a saved experiment with the optimizer (run from the repository root; repeat
mode needs no extra packages, while searches need `pip install optuna`; see `--help`):

```powershell
python autodrive_devkit/autodrive_roboracer/optimize_controller.py --repeat-trial 4 --from-run <run folder> --set front_slowdown_distance=1.5 --repeat-count 5
```
