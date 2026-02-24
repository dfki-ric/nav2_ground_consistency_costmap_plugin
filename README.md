
# nav2_ground_consistency_costmap_plugin

## Overview

`nav2_ground_consistency_costmap_plugin` is a custom Nav2 costmap layer that fuses
**ground** and **non-ground** point cloud evidence to produce a probabilistic
occupancy estimate in the costmap.

It integrates ground segmentation output directly into Nav2 and provides:

- Evidence-based occupancy scoring
- Temporal decay of observations
- Height-aware obstacle filtering
- Tunnel and small obstacle handling
- Rolling window support
- Thread-safe integration of point cloud callbacks

---

## Topics

The plugin subscribes to:

| Topic | Type | Description |
|-------|------|------------|
| `/ground_points` | `sensor_msgs/msg/PointCloud2` | Ground segmented points |
| `/nonground_points` | `sensor_msgs/msg/PointCloud2` | Obstacle segmented points |

Both clouds are transformed into the costmap global frame using TF.

---

## Parameters

| Parameter | Default | Description |
|-----------|----------|------------|
| `ground_points_topic` | `/ground_points` | Ground cloud topic |
| `nonground_points_topic` | `/nonground_points` | Obstacle cloud topic |
| `ground_inc` | `1.0` | Ground score increment per point |
| `nonground_inc` | `2.0` | Obstacle score increment per point |
| `ground_decay` | `0.90` | Ground score decay factor |
| `nonground_decay` | `0.98` | Obstacle score decay factor |
| `nonground_occ_thresh` | `3.0` | Minimum obstacle score for lethal classification |
| `nonground_prob_thresh` | `0.7` | Minimum obstacle probability for lethal classification |
| `max_score` | `1000.0` | Score clamp limit |
| `min_clearance` | `0.10` | Minimum obstacle height considered blocking |
| `robot_height` | `1.2` | Height threshold for tunnel detection |
| `tf_timeout` | `0.1` | TF lookup timeout (seconds) |

---

## Occupancy Model

For each grid cell:

```
p_occ = nonground_score / (ground_score + nonground_score + epsilon)
```

The costmap value is derived from `p_occ` and mapped to a 0–252 cost range.

If obstacle evidence exceeds both:

- `nonground_occ_thresh`
- `nonground_prob_thresh`

then additional height checks determine whether the cell is:

- 🚧 `LETHAL_OBSTACLE`
- 🟢 `FREE_SPACE` (tunnel or small obstacle)
- ⚖️ Probabilistic cost

---

## Height-Based Filtering

The plugin maintains per-cell statistics:

- Ground average height
- Obstacle minimum height
- Obstacle maximum height

It computes:

```
step_height = obstacle_min - ground_avg
obstacle_height = obstacle_max - ground_avg
```

This enables:

- Small obstacle suppression
- Tunnel clearance detection
- Step filtering

---

## Build

```bash
colcon build --packages-select nav2_ground_consistency_costmap_plugin
source install/setup.bash
```

---

## Nav2 Configuration Example

```yaml
local_costmap:
  plugins:
    - {name: ground_consistency_layer, type: "nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer"}

  ground_consistency_layer:
    ground_points_topic: "/ground_points"
    nonground_points_topic: "/nonground_points"
    ground_inc: 1.0
    nonground_inc: 2.0
    ground_decay: 0.90
    nonground_decay: 0.98
    nonground_occ_thresh: 3.0
    nonground_prob_thresh: 0.7
    min_clearance: 0.1
    robot_height: 1.2
```

---

## Dependencies

- ROS 2 (Humble or newer recommended)
- Nav2
- pluginlib
- tf2
- sensor_msgs
- geometry_msgs

---

## License

TODO: Add license.

---