# nav2_ground_consistency_costmap_plugin
![ground_consistency_layer](images/ground_consistency_layer.gif)

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
| `nonground_inc` | `1.5` | double | Evidence increment per obstacle point per update |
| `ground_decay` | `0.92` | double | Multiplicative decay factor for ground evidence per costmap update (0.0–1.0) |
| `nonground_decay` | `0.90` | double | Multiplicative decay factor for obstacle evidence per costmap update (0.0–1.0) |
| `nonground_occ_thresh` | `2.0` | double | Minimum obstacle score required to trigger height-based classification |
| `nonground_prob_thresh` | `0.750` | double | Minimum obstacle probability (0.0–1.0) required for lethal/free classification |
| `max_score` | `1000.0` | double | Upper clamp limit for score accumulation |
| `min_clearance` | `0.1` | double | Maximum obstacle height [m] considered "small" (passable under) |
| `robot_height` | `1.2` | double | Robot height [m] used for tunnel detection (obstacles above this are tunnels) |
| `tf_timeout` | `0.1` | double | TF lookup timeout [seconds] for cloud transformation |
| `footprint_clearing_enabled` | `true` | bool | Enable clearing of robot footprint polygon to prevent self-blocking |
| `enable_kpi_logging` | `false` | bool | Enable KPI tracking and CSV logging to `/tmp/costmap_kpi_*.csv` |

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
colcon build --packages-select nav2_ground_consistency_costmap_plugin --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
source install/setup.bash
```

---

## Nav2 Configuration Example

Add the layer to your Nav2 costmap configuration:

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
      min_clearance: 0.1              # 10 cm: small obstacles max height
      robot_height: 1.2               # 1.2 m: tunnel detection threshold
      tf_timeout: 0.1
      footprint_clearing_enabled: true # Clear robot footprint polygon
      enable_kpi_logging: false
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      # ... standard inflation layer config ...
```

**Note**: This layer can run standalone with its own footprint clearing, or alongside `obstacle_layer`
for complementary raw sensor data feedback. The footprint clearing prevents the robot from
self-blocking when traversing difficult terrain detected by ground segmentation.

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

## Design Rationale

### Clearing Strategy: Passive Decay vs. Active Raytracing

This plugin uses **passive temporal decay** for clearing, not active raytracing.

**Why passive decay is appropriate for this plugin:**

1. **Pre-segmented input assumption**: Ground points represent confirmed ground, implying
   obstacle-free space underneath. This is stronger than raw sensor data where raytracing
   is necessary to confirm free space.

2. **Height-aware classification**: Unlike obstacle_layer, we use ground height statistics
   to distinguish passable obstacles (tunnels, curbs). A decayed obstacle score doesn't
   make terrain unpredictable; height statistics remain to inform classification.

3. **Computational efficiency**: Raytracing all point clouds would be O(n * m) per cycle
   where n=points, m=costmap cells. Our O(n) evidence accumulation + O(k) decay is more
   efficient for dense point clouds.

4. **Dynamic environment handling**: Decay naturally "forgets" stale obstacles:
   - `ground_decay=0.92`: Ground evidence decays slower (environment stable)
   - `nonground_decay=0.90`: Obstacle evidence decays faster (cautious)
   Users can tune decay rates for their environment dynamics.

### Input Format: PointCloud2-Only (By Design)

This plugin **only accepts PointCloud2**, not LaserScan.

**Why PointCloud2, not LaserScan:**

1. **Alignment with Segmentation Pipeline**: Ground segmentation algorithms (KITTI, MLS-based)
   produce PointCloud2 output with semantic labels. LaserScan is unstructured range data
   requiring angle-based reconstruction. Supporting it adds no value and increases complexity.

2. **Upstream Conversion is Simpler**: Converting LaserScan→PointCloud2 is a well-solved
   problem. This follows Unix philosophy: do one thing well. The conversion responsibility
   belongs in the segmentation pipeline, not in this layer.

3. **No Performance Loss**: Point clouds from LaserScan conversion are just as efficient as
   native point clouds. The layer processes all PointCloud2 data uniformly.

4. **Reduces Plugin Complexity**: Adding LaserScan support requires:
   - LaserProjector dependency
   - Message type dispatch logic
   - Separate callback paths
   - Additional test cases

   For minimal benefit (a conversion that should happen upstream anyway).

**For users with LaserScans:**

Use standard ROS tooling to convert LaserScan to PointCloud2:
```bash
ros2 run laser_geometry laserscan_to_pointcloud_node \
  --ros-args -r scan_in:=/scan -r cloud_out:=/ground_points
```

Or integrate a conversion node into your launch pipeline.

**Design Philosophy**: This layer is a consumer of segmentation output, not a sensor driver.

### Footprint Clearing: Preventing Robot Self-Blocking

This plugin includes **active robot footprint clearing** to ensure the robot never self-blocks.

**How it works:**

1. **Every costmap update**, the layer retrieves the robot's footprint polygon
2. **Transforms** it to the current robot pose (robot_x, robot_y, robot_yaw)
3. **Marks all cells within the footprint** as FREE_SPACE to prevent obstacles from being placed under the robot
4. This runs **before** occupancy evidence is applied, ensuring footprint cells are always clear

**Footprint Configuration:**

- If a footprint is configured in your costmap (via Nav2 parameter `footprint`), it will be used
- If no footprint is configured then the clearance operation will be skipped.
- To customize, add this to your costmap config:
  ```yaml
  costmap:
    footprint: "[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]"  # meters
  ```

**Parameter Control:**

Set `footprint_clearing_enabled: false` in the layer config to disable this feature (not recommended for safety).

---

## License

See LICENSE file.

---

## Contributing

Contributions and bug reports are welcome. Please ensure code passes linting and tests before submission.

---
