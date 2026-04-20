# nav2_ground_consistency_costmap_plugin

![ground_consistency_layer](images/ground_consistency_layer.gif)

## Overview

A Nav2 costmap layer that fuses ground and non-ground point clouds into occupancy estimates. It classifies obstacles using height-based filtering: overhead structures (tunnels) appear free, small objects (curbs) can be ignored, and actual blocking obstacles trigger avoidance.

**Requirements:** Ground segmentation output (two PointCloud2 topics). Use [DFKI's ground_segmentation_ros2](https://github.com/dfki-ric/ground_segmentation_ros2) for LiDAR ground plane estimation.

## Quick Start

Minimal config:
```yaml
local_costmap:
  local_costmap:
    plugins: ["ground_consistency_layer", "inflation_layer"]
    ground_consistency_layer:
      plugin: "nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer"
      ground_points_topic: "/ground_points"
      nonground_points_topic: "/nonground_points"
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 1.0
```

Full config (all parameters):
```yaml
local_costmap:
  local_costmap:
    plugins: ["ground_consistency_layer", "inflation_layer"]
    ground_consistency_layer:
      plugin: "nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer"
      # Input topics
      ground_points_topic: "/ground_points"
      nonground_points_topic: "/nonground_points"
      tf_timeout: 0.1
      
      # Robot dimensions
      robot_height: 1.2
      min_clearance: 0.1
      
      # Evidence accumulation
      ground_inc: 1.0
      nonground_inc: 1.5
      
      # Decay rates (per update cycle)
      ground_decay: 0.80
      nonground_decay: 0.93
      
      # Thresholds
      nonground_occ_thresh: 2.0
      nonground_prob_thresh: 0.750
      max_score: 5000.0
      
      # Performance & memory
      max_data_range: 50.0
      discretize_costs: false
      footprint_clearing_enabled: true
      
      # Gap interpolation
      ground_neighbor_search_cells: 0
      
      # Logging
      enable_kpi_logging: false
    
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 1.0
```

## Key Concepts

**Occupancy Decision:** For each grid cell, the layer accumulates evidence:
- Ground points increase ground confidence
- Obstacle points increase obstacle confidence  
- Scores decay over time; older evidence fades out

**Height Classification:** When a cell has high obstacle evidence, it's classified as:
- **FREE** if the obstacle is taller than the robot (overhead structure) or very small
- **LETHAL** if it's actually a blocking obstacle at robot height

**Ground Estimation:** For cells with no local ground data (gaps between ground regions):
- Uses average ground height from neighboring cells if available
- Otherwise marks conservatively as LETHAL

## Parameters

| Parameter | Default | Type | Description |
|-----------|---------|------|-------------|
| `ground_points_topic` | `/ground_points` | string | Topic for ground-classified point cloud |
| `nonground_points_topic` | `/nonground_points` | string | Topic for obstacle-classified point cloud |
| `tf_timeout` | `0.1` | double | TF lookup timeout (seconds) |
| `ground_inc` | `1.0` | float | Evidence increment per ground point |
| `nonground_inc` | `1.5` | float | Evidence increment per obstacle point |
| `ground_decay` | `0.80` | float | Per-cycle ground score decay (0.0-1.0) |
| `nonground_decay` | `0.93` | float | Per-cycle obstacle score decay (0.0-1.0) |
| `nonground_occ_thresh` | `2.0` | float | Minimum obstacle score to consider height filtering |
| `nonground_prob_thresh` | `0.750` | float | Minimum p_occ probability (0.0-1.0) to classify cell |
| `max_score` | `5000.0` | double | Upper clamp for accumulated scores |
| `min_clearance` | `0.1` | double | Ignore obstacles with height < min_clearance (m) |
| `robot_height` | `1.2` | double | Robot height; taller obstacles classified as FREE (m) |
| `footprint_clearing_enabled` | `true` | bool | Clear evidence under robot footprint polygon each cycle |
| `enable_kpi_logging` | `false` | bool | Write cycle metrics to `/tmp/costmap_kpi_*.csv` |
| `max_data_range` | `50.0` | double | Retain cell data only within this distance from robot (0=disabled) |
| `discretize_costs` | `false` | bool | Output only binary costs (LETHAL or FREE, no gradients) |
| `ground_neighbor_search_cells` | `0` | int | Search radius (cell count) for neighbor ground height interpolation. 0=disabled

## Tuning

**Favor detecting obstacles more carefully** (current defaults):
- Low ground decay (0.80) → false-positive ground fades fast
- High nonground decay (0.93) → real obstacles stick around
- High max_score (5000) → allows obstacle dominance before saturation

**For more aggressive obstacle detection** (fewer false negatives):
- Lower `ground_decay` (e.g., 0.70-0.80) to fade false-positive ground faster
- Raise `nonground_decay` (e.g., 0.93-0.96) to preserve real obstacles longer
- Lower `nonground_prob_thresh` (e.g., 0.70) to require less obstacle confidence

**For more conservative obstacle detection** (fewer false positives):
- Raise `ground_decay` (e.g., 0.85-0.92) to preserve ground estimates longer
- Lower `nonground_decay` (e.g., 0.85-0.92) to fade old evidence faster
- Raise `nonground_prob_thresh` (e.g., 0.80) to require stronger evidence

**Dense lidar (many false-positive ground points at obstacle base)**:
- Lower `ground_decay` (0.70-0.80) to suppress stale false ground
- Raise `nonground_decay` (0.93-0.96) to let real obstacles accumulate

**Sparse lidar (few ground points, many gaps)**:
- Increase `ground_neighbor_search_cells` (e.g., 3-5 cells) to interpolate gap heights

**Memory pressure (many cells, large environment)**:
- Lower `max_data_range` (e.g., 30m instead of 50m)
- Disable KPI logging (`enable_kpi_logging: false`)

**Performance (high CPU usage)**:
- Enable `discretize_costs: true` to skip probabilistic cost mapping
- Reduce `max_data_range` to cull distant cells sooner

## Build

```bash
colcon build --packages-select nav2_ground_consistency_costmap_plugin
source install/setup.bash
```

## Topics

- **Input:** `/ground_points` and `/nonground_points` (PointCloud2)
- **Output:** Costmap layer

## License

See LICENSE file.

## Funding

Developed at the Robotics Innovation Center (DFKI), Bremen. Supported by Robdekon2 (50RA1406), German Federal Ministry for Research and Technology.
