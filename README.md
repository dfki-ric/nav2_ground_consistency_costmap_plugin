<div align="center">
  <h1>Nav2 Ground Consistency Costmap Plugin</h1>
  <a href="https://github.com/dfki-ric/ground_segmentation">Core Library</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://github.com/dfki-ric/ground_segmentation_ros2">ROS 2 Wrapper</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://docs.nav2.org/plugins/index.html">Nav2 Plugin Docs</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://doi.org/10.1109/ICRCV67407.2025.11349133">Paper</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://docs.nav2.org/tutorials/index.html">Nav2 Tutorials</a>
</div>

---

![ground_consistency_layer](images/ground_consistency_layer.gif)

[Arter Excavator](https://robotik.dfki-bremen.de/en/research/robot-systems/arter) at the [Robotics Innovation Center, Bremen, Germany](https://robotik.dfki-bremen.de/en/startpage)

## Demo Scenarios

Let us see the performance on real-world data. We used [kiss-icp](https://github.com/prbonn/kiss-icp) as source of odometry.

[BotanicGarden dataset](https://github.com/robot-pesg/BotanicGarden)
<p align="center">
  <img src="images/botanic_garden.gif" alt="Botanic garden scenario" width="100%" />
</p>

[Citrus-Farm dataset](https://github.com/UCR-Robotics/Citrus-Farm-Dataset)

<p align="center">
  <img src="images/citrus_farm.gif" alt="Citrus farm scenario" width="100%" />
</p>

## Overview

A Nav2 costmap layer that fuses ground and non-ground point clouds into occupancy estimates. It classifies obstacles using height-based filtering: overhead structures (tunnels) appear free, small objects (curbs) can be ignored, and actual blocking obstacles trigger avoidance.

**Requirements:** Ground segmentation output (two PointCloud2 topics). Use [DFKI's ground_segmentation_ros2](https://github.com/dfki-ric/ground_segmentation_ros2) for LiDAR based ground segmentation.

## Ground Consistency Layer

Ground Consistency is a costmap layer that leverages ground segmentation to create more reliable occupancy estimates for navigation in challenging outdoor environments. While it can be used with any ground segmentation algorithm, we recommend using GSeg3D for which this plugin was originally developed to integrate with.

**Key Features:**

- **Evidence-Based Probabilistic Approach**: Maintains accumulated evidence of ground and obstacle points across multiple sensor observations, rather than making binary decisions on individual measurements
- **Evidence Competition**: Ground and obstacle points compete to determine the true occupancy status of each costmap cell, reducing false positives and false negatives
- **Height-Based Classification**: Distinguishes between actual obstacles and terrain variations (e.g., slopes, small bumps) by evaluating obstacle height relative to local ground level
- **Temporal Stability**: Evidence accumulates and decays over time, creating smooth transitions between free and occupied states while maintaining responsiveness to environmental changes
- **Noise Resilience**: Protects against isolated sensor noise by requiring sustained evidence before marking a cell as occupied

### 1. Evidence Accumulation and Competition System

Each grid cell in the layer maintains two types of evidence: ground and obstacle. As new sensor data arrives, these scores are updated and compared to estimate how likely the cell is occupied. Evidence weights for ground and obstacle points can be adjusted independently (e.g., obstacle evidence may be weighted more heavily than ground evidence) to create a safety bias.

A cell is marked as an obstacle only when there is both:

- enough evidence of obstacle points, and
- high confidence that the evidence of obstacle is stronger than the evidence of ground.

This approach prevents isolated sensor noise from affecting navigation. For example, a single false positive obstacle point will not mark a cell as occupied if there is strong ground evidence.

### 2. Height-Based Occupancy Classification

Not all detected obstacles actually block the robot. The layer evaluates obstacle height relative to the local ground height. Based on the robot's height:

- Very high objects are treated as overhead (safe to pass under)
- Very low objects are treated as terrain variation
- Only objects within the robot's collision range are considered blocking

At times, the terrain is such that no local ground height can be reliably determined. In this case, the layer can be configured to use neighboring cells to estimate local ground height (see the `ground_neighbor_search_cells` parameter), or treat all such obstacles without ground below them as blocking. For example, if the robot is navigating through a tunnel and the ground segmentation fails to detect any ground points, then as a backup plan, a `maximum_height_filter` can be applied to incoming obstacle points. This allows the robot to navigate through tunnels and under bridges without being blocked by misclassified ground points.

### 3. Temporal Stability Through Evidence Decay

Evidence is decayed over time to allow the costmap to adapt to changing environments. Cells transition gradually between free and occupied states as evidence builds or fades. The rate of decay can be tuned separately for ground and obstacle evidence, creating temporal hysteresis, which allows for more stable and responsive terrain adaptation (faster ground decay) while maintaining stable obstacle marking (slower obstacle decay).

## Configuration
It is recommended to use the layer in the local costmap
since it relies on real-time sensor data and is designed for short-term occupancy estimation. For safety, use the layer
together with the inflation layer to create a buffer around detected obstacles.

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
      maximum_height_filter: 5.0
      
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
| `maximum_height_filter` | `inf` | double | Filter out points above this height; useful for ignoring ceilings/overhangs (m, 0=disabled) |
| `footprint_clearing_enabled` | `true` | bool | Clear evidence under robot footprint polygon each cycle |
| `enable_kpi_logging` | `false` | bool | Write cycle metrics to `/tmp/costmap_kpi_*.csv` |
| `max_data_range` | `50.0` | double | Retain cell data only within this distance from robot (0=disabled) |
| `discretize_costs` | `false` | bool | Output only binary costs (LETHAL or FREE, no gradients) |
| `ground_neighbor_search_cells` | `0` | int | Search radius (cell count) for neighbor ground height interpolation. 0=disabled

## Tuning

**Height Filtering (Overhead Obstacles):**
- Set `maximum_height_filter` to prevent ceiling/overhead structures from blocking navigation
- Useful for indoor navigation where lidar may detect ceilings wrongly as obstacles

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

## License

See LICENSE file.

## Funding

Developed at the Robotics Innovation Center (DFKI), Bremen. Supported by Robdekon2 (50RA1406), German Federal Ministry for Research and Technology.
