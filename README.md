# nav2_ground_consistency_costmap_plugin

![ground_consistency_layer](images/ground_consistency_layer.gif)

## Overview

A Nav2 costmap layer that fuses ground and non-ground point clouds into occupancy estimates. It classifies obstacles using height-based filtering: overhead structures (tunnels) appear free, small objects (curbs) can be ignored, and actual blocking obstacles trigger avoidance.

**Requirements:** Ground segmentation output (two PointCloud2 topics). Use [DFKI's ground_segmentation_ros2](https://github.com/dfki-ric/ground_segmentation_ros2) for LiDAR ground plane estimation.

## Quick Start

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

## Key Concepts

**Occupancy Decision:** For each grid cell, the layer accumulates evidence:
- Ground points increase ground confidence
- Obstacle points increase obstacle confidence  
- Scores decay over time; older evidence fades out

**Height Classification:** When a cell has high obstacle evidence, it's classified as:
- **FREE** if the obstacle is taller than the robot (overhead structure) or very small (<10cm)
- **LETHAL** if it's actually a blocking obstacle at robot height

**Ground Estimation:** For cells with no local ground data (gaps between ground regions):
- Uses cellular ground height if available
- Otherwise marks conservatively as LETHAL

## Common Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ground_points_topic` | `/ground_points` | Incoming ground points |
| `nonground_points_topic` | `/nonground_points` | Incoming obstacle points |
| `robot_height` | `1.2` | Robot height for tunnel detection (m) |
| `min_clearance` | `0.1` | Ignore obstacles smaller than this (m) |
| `ground_decay` | `0.80` | Ground evidence fade rate per cycle (0.0-1.0) |
| `nonground_decay` | `0.93` | Obstacle evidence fade rate per cycle (0.0-1.0) |
| `max_score` | `5000.0` | Accumulation ceiling _before saturation_ |
| `max_data_range` | `50.0` | Keep cells only within 50m of robot (0=disabled) |
| `footprint_clearing_enabled` | `true` | Clear evidence under robot footprint |
| `enable_kpi_logging` | `false` | Write metrics to `/tmp/costmap_kpi_*.csv` |

Full parameter list: see source code `onInitialize()`.

## Tuning

**Favor detecting obstacles more carefully** (current defaults):
- Low ground decay (0.80) → false-positive ground fades fast
- High nonground decay (0.93) → real obstacles stick around
- High max_score (5000) → allows obstacle evidence to dominate

**For more aggressive traversal**, increase `robot_height` or `min_clearance` to classify more structures as passable.

**For more conservative navigation**, decrease decay factors to make old obstacles stick longer.

## Build

```bash
colcon build --packages-select nav2_ground_consistency_costmap_plugin
source install/setup.bash
```

## Topics

- **Input:** `/ground_points` and `/nonground_points` (PointCloud2)
- **Output:** Costmap layer (Grid2D)

## License

See LICENSE file.

## Funding

Developed at the Robotics Innovation Center (DFKI), Bremen. Supported by Robdekon2 (50RA1406), German Federal Ministry for Research and Technology.
