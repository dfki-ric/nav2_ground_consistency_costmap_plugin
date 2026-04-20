# nav2_ground_consistency_costmap_plugin

![ground_consistency_layer](images/ground_consistency_layer.gif)

## Overview

A Nav2 costmap layer that fuses ground and non-ground point cloud evidence into an occupancy estimate.

It consumes the output of a ground segmentation node (two PointCloud2 topics) and integrates it directly into Nav2's costmap stack. The layer accumulates per-cell evidence scores across frames and decays them over time, providing stable occupancy estimates suitable for navigation planning. Ground height statistics are used to classify obstacles as blocking, passable (small), or overhead (tunnels).

## Ground Segmentation

This layer requires a ground segmentation algorithm to separate ground and non-ground points. Consider using [DFKI's ground_segmentation_ros2](https://github.com/dfki-ric/ground_segmentation_ros2) for robust ground plane estimation from LiDAR data.

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
| `ground_decay` | `0.80` | Per-cycle multiplicative decay for ground evidence (0.0-1.0) |
| `nonground_decay` | `0.93` | Per-cycle multiplicative decay for obstacle evidence (0.0-1.0) |
| `nonground_occ_thresh` | `2.0` | Minimum obstacle score to trigger height classification |
| `nonground_prob_thresh` | `0.750` | Minimum obstacle probability for lethal/free classification |
| `max_score` | `5000.0` | Upper clamp for score accumulation |
| `min_clearance` | `0.1` | Max obstacle height (m) considered passable |
| `robot_height` | `1.2` | Robot height (m) for tunnel detection |
| `tf_timeout` | `0.1` | TF lookup timeout (seconds) |
| `footprint_clearing_enabled` | `true` | Clear evidence under the robot footprint polygon |
| `max_data_range` | `50.0` | Max distance (m) from robot to retain cell data. `0` = disabled (use costmap window only) |
| `ground_neighbor_search_radius` | `0` | Neighbor interpolation radius (m) for gap cells. `0` = disabled (only use Tier 1 local ground) |
| `enable_kpi_logging` | `false` | Write per-cycle metrics to `/tmp/costmap_kpi_*.csv` |
| `discretize_costs` | `false` | Output binary costs (LETHAL or FREE only) instead of probabilistic gradients |

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

### 3-Tier Ground Reference System

For cells without local ground data (gap cells between ground regions), the layer estimates ground height using a three-tier fallback:

**Tier 1 (Local):** Use cell's own ground height statistics if available.

**Tier 2 (Neighbor Interpolation):** If local data missing, average ground height from neighboring cells within `ground_neighbor_search_radius`. Disabled by default (`ground_neighbor_search_radius: 0`).

**Tier 3 (Conservative Lethal):** If no neighbors found, mark cell as LETHAL to prevent false passages through unmapped gaps.

This avoids brittle assumptions (e.g., robot Z coordinate) while still enabling traversable gap classification.

### Per-Frame Height Tracking

To prevent stray sensor noise from permanently corrupting obstacle height estimates, the layer uses per-frame height buffers:

- During point cloud callback: update frame-local min/max height (`obstacle_min_height_frame`, `obstacle_max_height_frame`)
- During `updateBounds()`: refresh persistent heights from current frame data, reset frame buffers

Result: cells recover when noisy points disappear, avoiding permanent contamination from single outliers.

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
      ground_decay: 0.80
      nonground_decay: 0.93
      nonground_occ_thresh: 2.0
      nonground_prob_thresh: 0.750
      max_score: 5000.0
      min_clearance: 0.1
      robot_height: 1.2
      tf_timeout: 0.1
      footprint_clearing_enabled: true
      max_data_range: 50.0
      ground_neighbor_search_radius: 0
      enable_kpi_logging: false
      discretize_costs: false
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
```

This layer serves as an alternative to `obstacle_layer` for systems with ground segmentation. It is typically used alongside `inflation_layer`.

## Alternative: Non-Persistent Voxel Layer

For systems requiring stateless, non-accumulating obstacle detection (e.g., poor odometry), consider using Nav2's **Non-Persistent Voxel Layer (NPVL)** instead. NPVL processes only the current sensor frame without evidence accumulation, making it lightweight and responsive to immediate sensor changes. Refer to [Nav2 NPVL documentation](https://github.com/ros-planning/navigation2/tree/humble/nav2_costmap_2d) for configuration.

## Dependencies

- ROS 2 Jazzy or newer (not tested on Humble yet)
- nav2_costmap_2d
- pluginlib
- tf2 / tf2_ros
- sensor_msgs
- geometry_msgs

## Tuning Guide

### Recent Tuning (v2: Pillar Detection)

The default parameters have been tuned to improve detection of thin vertical obstacles (e.g., pillars) and reduce false positive clearance detection at close range:

- **`ground_decay: 0.80`** (was 0.92): False-positive ground points at obstacle bases fade 2x faster (20% per cycle vs 8%), allowing nonground evidence to dominate over time.
- **`nonground_decay: 0.93`** (was 0.90): Real obstacle evidence persists longer (7% per cycle fade), improving continuity during partial occlusions.
- **`max_score: 5000.0`** (was 1000.0): Higher accumulation ceiling allows nonground evidence (scaled at 1.5x) to express dominance over ground evidence (scaled at 1.0x) before both saturate.
- **`ground_neighbor_search_radius: 0`** (was 2): Disabled neighbor interpolation for gap cells. Conservative approach: cells without local ground data are marked LETHAL rather than interpolated from neighboring ground cells. Reduces false-positive free space in unmapped gaps.
- **`max_data_range: 50.0`** (was 0.0): Retain evidence only within 50m of robot to bound memory growth and focus on relevant navigation obstacles.

These settings significantly improve reliability for obstacle detection with dense LiDAR (128 vertical samples) at high update rates (20 Hz), especially in complex environments with dense false-positive ground classification from nearby structures.

### Occupancy Sensitivity

To increase traversable area (fewer obstacles):
- Decrease `robot_height` (more areas classified as tunnels)
- Increase `min_clearance` (more small obstacles ignored)
- Lower `nonground_occ_thresh` or `nonground_prob_thresh`
- Increase decay factors (evidence persists longer)

To increase obstacle avoidance (more conservative):
- Increase `robot_height` (fewer tunnels)
- Decrease `min_clearance` (fewer small obstacles ignored)
- Raise `nonground_occ_thresh` or `nonground_prob_thresh`
- Decrease decay factors (evidence fades faster)

## Design Decisions

### Why Passive Decay Instead of Raytracing

The layer uses temporal decay to clear stale evidence rather than active raytracing. Ground points already represent confirmed ground, which implies obstacle-free space. Height statistics persist independently of score decay, so classification remains informed even as scores fade. This is computationally cheaper than raytracing (O(n) accumulation + O(k) decay vs O(n*m) raycasting) and naturally handles dynamic environments through tunable decay rates.

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
