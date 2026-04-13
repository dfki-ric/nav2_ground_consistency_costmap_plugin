# nav2_ground_consistency_costmap_plugin

![ground_consistency_layer](images/ground_consistency_layer.gif)

## Overview

A Nav2 costmap layer that fuses ground and non-ground point cloud evidence into an occupancy estimate.

It consumes the output of a ground segmentation node (two PointCloud2 topics) and integrates it directly into Nav2's costmap stack. The layer supports two processing modes:

- **Temporal Mode** (default): Accumulates per-cell evidence scores across frames and decays them over time. Best for systems with good odometry and sparse sensor data.
- **One-Shot Mode**: Processes only the latest data frame without accumulation. Best for systems with poor odometry or dense sensor data.

Both modes use ground height statistics to classify obstacles as blocking, passable (small), or overhead (tunnels).

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ground_points` | `sensor_msgs/msg/PointCloud2` | Ground-classified points from a segmentation node |
| `/nonground_points` | `sensor_msgs/msg/PointCloud2` | Non-ground/obstacle-classified points |

Both clouds are transformed into the costmap's global frame via TF2.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ground_points_topic` | `/ground_points` | Topic for ground point cloud |
| `nonground_points_topic` | `/nonground_points` | Topic for obstacle point cloud |
| `ground_inc` | `1.0` | Evidence added per ground point |
| `nonground_inc` | `1.5` | Evidence added per obstacle point |
| `ground_decay` | `0.92` | Per-cycle multiplicative decay for ground evidence (0.0-1.0) |
| `nonground_decay` | `0.90` | Per-cycle multiplicative decay for obstacle evidence (0.0-1.0) |
| `nonground_occ_thresh` | `2.0` | Minimum obstacle score to trigger height classification |
| `nonground_prob_thresh` | `0.750` | Minimum obstacle probability for lethal/free classification |
| `max_score` | `1000.0` | Upper clamp for score accumulation |
| `min_clearance` | `0.1` | Max obstacle height (m) considered passable |
| `robot_height` | `1.2` | Robot height (m) for tunnel detection |
| `tf_timeout` | `0.1` | TF lookup timeout (seconds) |
| `footprint_clearing_enabled` | `true` | Clear evidence under the robot footprint polygon |
| `max_data_range` | `0.0` | Max distance (m) from robot to retain cell data. `0` = disabled (use costmap window only) |
| `enable_kpi_logging` | `false` | Write per-cycle metrics to `/tmp/costmap_kpi_*.csv` |
| `one_shot_mode` | `false` | Process only latest data frame (true) vs accumulate evidence across frames (false) |
| `discretize_costs` | `false` | Output binary costs: LETHAL or FREE only (true) vs probabilistic gradients (false) |

## Occupancy Model

Each grid cell stores a single `CellData` struct containing ground and obstacle evidence scores, ground height statistics (sum and count), and obstacle height bounds (min and max z).

The occupancy probability is:

```
p_occ = nonground_score / (ground_score + nonground_score + epsilon)
```

This is mapped linearly to the costmap cost range (0-252).

### Height-Based Classification

When obstacle evidence exceeds both thresholds (`nonground_score >= nonground_occ_thresh` and `p_occ >= nonground_prob_thresh`), the layer computes:

```
step_height     = obstacle_min_z - avg_ground_z
obstacle_height = obstacle_max_z - avg_ground_z
```

| Condition | Cost | Reason |
|-----------|------|--------|
| `step_height > robot_height` | FREE_SPACE | Overhead structure, robot fits underneath |
| `obstacle_height < min_clearance` | FREE_SPACE | Small obstacle (curbs, low vegetation) |
| Otherwise | LETHAL_OBSTACLE | Blocking obstacle |

### No-Data Decay Guard

When no sensor data arrives (e.g., bag pause, sensor dropout), decay is skipped entirely. The costmap retains its last known state rather than silently erasing all evidence.

## Data Structure

All per-cell data is stored in a single `std::unordered_map<WorldKey, CellData>`. The WorldKey is a 64-bit packed pair of signed 32-bit grid coordinates in the global frame. This means cells are stable across costmap rolling window shifts without requiring any data migration.

The hash map is pre-allocated in `matchSize()` to the costmap cell count to avoid rehashing during operation.

Cells outside the active costmap window are erased each cycle. Cells whose scores decay below a noise threshold (1e-3) are also removed, preventing unbounded memory growth.

## Processing Pipeline

Point cloud callbacks run asynchronously and perform a single pass per cloud: each point is transformed inline using `tf2::Transform`, quantized to a WorldKey, and accumulated into the cell's frame counters. There is no intermediate cloud copy or multi-pass iteration.

During `updateBounds()`, all heavy computation happens in a single pass over the cell map: frame counters are integrated into persistent scores, cells beyond `max_data_range` or outside the costmap window are culled, decay is applied (if new data arrived), costs are computed and stored, and bounds are updated. During `updateCosts()`, footprint evidence is cleared using a point-in-polygon test, and pre-computed costs are written to the master grid via raw pointer access.

All shared state is protected by a mutex.

## Build

```bash
colcon build --packages-select nav2_ground_consistency_costmap_plugin
source install/setup.bash
```

## Configuration Example

```yaml
local_costmap:
  local_costmap:
    plugins: ["ground_consistency_layer", "inflation_layer"]
    ground_consistency_layer:
      plugin: "nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer"
      ground_points_topic: "/ground_points"
      nonground_points_topic: "/nonground_points"
      ground_inc: 1.0
      nonground_inc: 1.5
      ground_decay: 0.92
      nonground_decay: 0.90
      nonground_occ_thresh: 2.0
      nonground_prob_thresh: 0.750
      max_score: 1000.0
      min_clearance: 0.1
      robot_height: 1.2
      tf_timeout: 0.1
      footprint_clearing_enabled: true
      max_data_range: 0.0
      enable_kpi_logging: false
      one_shot_mode: false
      discretize_costs: false
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
```

This layer serves as an alternative to `obstacle_layer` for systems with ground segmentation. It is typically used alongside `inflation_layer`.

## Processing Modes

### Temporal Mode (Default)

Evidence accumulates across frames and decays over time:
- **Score integration**: `score += count * increment`
- **Decay (per frame)**: `score *= decay_factor` (only when new data arrives)
- **Response**: Slow but stable; needs multiple scans to exceed thresholds
- **Best for**: Good odometry, sparse sensors, need for stable estimates
- **Configuration**: `one_shot_mode: false`, `ground_decay: 0.92`, `nonground_decay: 0.90`

### One-Shot Mode

Processes only the current frame's data without accumulation:
- **Score integration**: `score = count * increment` (replaces old score)
- **Decay**: None applied
- **Callback sync**: Only processes when both ground and nonground callbacks have fired
- **Response**: Instant; single frame with sufficient points can trigger detection
- **Best for**: Poor odometry, dense sensors, immediate response needed
- **Advantage for bad odometry**: Fresh data only per frame means wrong transformations disappear next cycle instead of persisting
- **Configuration**: `one_shot_mode: true`, higher `nonground_occ_thresh` and `nonground_prob_thresh` (e.g., 5.0 and 0.90)

## Dependencies

- ROS 2 Jazzy or newer (not tested on Humble yet)
- nav2_costmap_2d
- pluginlib
- tf2 / tf2_ros
- sensor_msgs
- geometry_msgs

## Tuning Guide

### Mode Selection

**Use Temporal Mode if**:
- Robot odometry is stable and accurate
- Sensor provides sparse data
- Need smooth, stable costmap
- Can tolerate slightly delayed obstacle detection

**Use One-Shot Mode if**:
- Robot odometry is poor or uncertain
- Sensor provides dense point clouds per frame
- Need immediate response to new obstacles
- Can tolerate flickering if callbacks are delayed

### Occupancy Sensitivity

To increase traversable area (fewer obstacles):
- Decrease `robot_height` (more areas classified as tunnels)
- Increase `min_clearance` (more small obstacles ignored)
- Lower `nonground_occ_thresh` or `nonground_prob_thresh`
- In temporal mode: Increase decay factors (evidence persists longer)

To increase obstacle avoidance (more conservative):
- Increase `robot_height` (fewer tunnels)
- Decrease `min_clearance` (fewer small obstacles ignored)
- Raise `nonground_occ_thresh` or `nonground_prob_thresh`
- In temporal mode: Decrease decay factors (evidence fades faster)

### One-Shot Mode Tuning

In one-shot mode, set conservative thresholds to avoid false positives:
- `nonground_occ_thresh: 4.0-5.0` (requires 3+ points per frame)
- `nonground_prob_thresh: 0.85-0.90` (requires strong dominance)
- Use `discretize_costs: true` for clean binary output

## Design Decisions

### Why Passive Decay Instead of Raytracing

In temporal mode, the layer uses temporal decay to clear stale evidence rather than active raytracing. Ground points already represent confirmed ground, which implies obstacle-free space. Height statistics persist independently of score decay, so classification remains informed even as scores fade. This is computationally cheaper than raytracing (O(n) accumulation + O(k) decay vs O(n*m) raycasting) and naturally handles dynamic environments through tunable decay rates.

In one-shot mode, decay is not applied; instead, evidence is replaced each frame, providing immediate responsiveness to sensor changes.

### Why PointCloud2 Only

Ground segmentation algorithms produce PointCloud2 output. LaserScan support would add complexity (LaserProjector dependency, separate callback paths, message dispatch) for no practical benefit. Users with LaserScan data should convert upstream.

### Footprint Clearing

Each update cycle, the layer clears evidence under the robot's footprint polygon to prevent self-blocking. A point-in-polygon test ensures only cells actually under the footprint are cleared, not the entire bounding box. The footprint is obtained from Nav2's costmap configuration. Set `footprint_clearing_enabled: false` to disable.

## KPI Logging

When `enable_kpi_logging` is set to `true`, the layer writes per-cycle metrics to `/tmp/costmap_kpi_<layer_name>.csv`. A plotting script is included:

```bash
python3 scripts/plot_costmap_kpi.py
```

This produces charts for cycle latency, cells updated/decayed, map sizes, memory usage, and points processed per update.

## License

See LICENSE file.

## Contributing

Contributions and bug reports are welcome.

## Funding

This package was initiated and is currently developed at the Robotics Innovation Center of the German Research Center for Artificial Intelligence (DFKI) in Bremen. The development was started in the scope of Robdekon2 (50RA1406), funded by the German Federal Ministry for Research, Technology, and Space.
