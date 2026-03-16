
# nav2_ground_consistency_costmap_plugin

## Overview

`nav2_ground_consistency_costmap_plugin` is a custom Nav2 costmap layer that fuses
**ground** and **non-ground** point cloud evidence to produce a probabilistic
occupancy estimate in the costmap with temporal decay and height-aware filtering.

It integrates ground segmentation output directly into Nav2's costmap stack and provides:

- **Evidence-based occupancy scoring**: Accumulates evidence from ground and obstacle points
- **Temporal decay**: Scores decay over time to handle out-of-date observations
- **Height-aware obstacle filtering**: Distinguishes between tunnels, small obstacles, and true blocking obstacles
- **Per-cell terrain statistics**: Tracks ground height and obstacle dimensions for intelligent classification
- **Rolling window support**: Efficiently manages costmap updates with automatic memory management
- **Thread-safe integration**: Asynchronous point cloud callbacks with mutex protection

---

## Topics

The plugin subscribes to two point cloud topics:

| Topic | Type | Description |
|-------|------|------------|
| `/ground_points` | `sensor_msgs/msg/PointCloud2` | Ground-classified points (e.g., from segmentation node) |
| `/nonground_points` | `sensor_msgs/msg/PointCloud2` | Non-ground/obstacle-classified points |

Both clouds are automatically transformed into the costmap's global frame using TF2.

---

## Parameters

Configuration parameters control the evidence accumulation, decay rates, and classification thresholds:

| Parameter | Default | Type | Description |
|-----------|---------|------|------------|
| `ground_points_topic` | `/ground_points` | string | Topic name for ground point cloud |
| `nonground_points_topic` | `/nonground_points` | string | Topic name for obstacle point cloud |
| `ground_inc` | `1.0` | double | Evidence increment per ground point per update |
| `nonground_inc` | `2.0` | double | Evidence increment per obstacle point per update |
| `ground_decay` | `0.90` | double | Multiplicative decay factor for ground evidence per costmap update (0.0–1.0) |
| `nonground_decay` | `0.98` | double | Multiplicative decay factor for obstacle evidence per costmap update (0.0–1.0) |
| `nonground_occ_thresh` | `3.0` | double | Minimum obstacle score required to trigger height-based classification |
| `nonground_prob_thresh` | `0.7` | double | Minimum obstacle probability (0.0–1.0) required for lethal/free classification |
| `max_score` | `1000.0` | double | Upper clamp limit for score accumulation |
| `min_clearance` | `0.10` | double | Maximum obstacle height [m] considered "small" (passable under) |
| `robot_height` | `1.2` | double | Robot height [m] used for tunnel detection (obstacles above this are tunnels) |
| `tf_timeout` | `0.1` | double | TF lookup timeout [seconds] for cloud transformation |

---

## Occupancy Model

Each grid cell maintains:
- **Ground evidence score**: Accumulated and decayed from ground points
- **Obstacle evidence score**: Accumulated and decayed from non-ground points
- **Ground height statistics**: Sum and count for computing average height
- **Obstacle height bounds**: Minimum and maximum z-coordinate of obstacles

The occupancy probability is computed as:

```
p_occ = nonground_score / (ground_score + nonground_score + epsilon)
```

### Cost Mapping

The probability `p_occ` is linearly mapped to the costmap cost value (0–252):
```
cost = p_occ × 252.0  (clamped to 0–252)
```

### Intelligent Classification

When obstacle evidence exceeds **both** thresholds (`nonground_score >= nonground_occ_thresh` AND `p_occ >= nonground_prob_thresh`):

Height-based filtering occurs using per-cell terrain statistics:

| Condition | Result | Reason |
|-----------|--------|--------|
| `step_height > robot_height` | 🟢 **FREE_SPACE** | Tunnel (can pass underneath) |
| `obstacle_height < min_clearance` | 🟢 **FREE_SPACE** | Small obstacle (curbs, low vegetation) |
| Otherwise | 🚧 **LETHAL_OBSTACLE** | Confirmed blocking obstacle |

Where:
```
step_height = obstacle_min_height - ground_avg_height
obstacle_height = obstacle_max_height - ground_avg_height
```

---

## Data Structure & Memory Management

The plugin uses efficient world-frame keying:

- **WorldKey**: 64-bit packed representation of world-frame grid coordinates
  - Enables stable, unique identification of cells across costmap rolling windows
  - Automatically sampled to costmap resolution

- **Per-cell maps** (unordered_map with WorldKey):
  - Ground scores, nonground scores
  - Ground height sums/counts, obstacle min/max heights

- **Rolling window cleanup**:
  - Cells outside the active window are removed
  - Scores below noise threshold (1e-3) are erased with associated data
  - Prevents unbounded memory growth

---

## Thread Safety

- **Asynchronous callbacks**: Ground and nonground cloud callbacks run in ROS 2's message thread
- **Mutex protection**: All shared data structures protected by `std::lock_guard<std::mutex>`
- **Callback ordering**: Frame counts accumulated during callbacks, integrated into scores during `updateCosts()`

---

## Build & Installation

### Build the package:

```bash
colcon build --packages-select nav2_ground_consistency_costmap_plugin
source install/setup.bash
```

---

## Nav2 Configuration Example

Add the layer to your Nav2 costmap configuration:

```yaml
local_costmap:
  local_costmap:
    plugins: ["static_layer", "obstacle_layer", "ground_consistency_layer", "inflation_layer"]
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: true

    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      # ... standard obstacle layer config ...

    ground_consistency_layer:
      plugin: "nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer"
      ground_points_topic: "/ground_points"
      nonground_points_topic: "/nonground_points"
      ground_inc: 1.0
      nonground_inc: 2.0
      ground_decay: 0.90
      nonground_decay: 0.98
      nonground_occ_thresh: 3.0
      nonground_prob_thresh: 0.7
      max_score: 1000.0
      min_clearance: 0.1       # 10 cm: small obstacles max height
      robot_height: 1.2        # 1.2 m: tunnel detection threshold
      tf_timeout: 0.1

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      # ... standard inflation layer config ...
```

---

## Dependencies

- **ROS 2**: Humble or newer (tested with Humble, Jazzy)
- **Nav2**: `nav2_costmap_2d` 1.1.0+
- **pluginlib**: For plugin loading
- **tf2 / tf2_ros**: Transform tree lookup
- **sensor_msgs**: PointCloud2 message type
- **geometry_msgs**: For transformation operations

---

## Tuning Guide

### Increasing traversable area (fewer obstacles):
- Decrease `robot_height` → more areas classified as tunnels
- Decrease `nonground_occ_thresh` or `nonground_prob_thresh` → lower obstacle evidence required
- Increase `min_clearance` → more obstacles classified as "small"
- Increase decay factors (`ground_decay`, `nonground_decay`) → evidence persists longer

### Increasing obstacle avoidance (more conservative):
- Increase `robot_height` → fewer tunnels
- Increase `nonground_occ_thresh` or `nonground_prob_thresh` → higher evidence required
- Decrease `min_clearance` → fewer small obstacles ignored
- Decrease decay factors → evidence decays faster

### Performance:
- If lagging, check point cloud frequency and density
- Adjust `max_score` to prevent score explosion
- Monitor memory with `ros2 node info /costmap_node`

---

## License

See LICENSE file or contact maintainer.

---

## Contributing

Contributions and bug reports are welcome. Please ensure code passes linting and tests before submission.

---