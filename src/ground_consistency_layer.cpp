#include "nav2_ground_consistency_costmap_plugin/ground_consistency_layer.hpp"

#include <algorithm>
#include <limits>
#include <cmath>
#include <string>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace nav2_ground_consistency_costmap_plugin
{

static inline int32_t unpackX(GroundConsistencyLayer::WorldKey k)
{
  return static_cast<int32_t>(static_cast<uint32_t>(k >> 32));
}

static inline int32_t unpackY(GroundConsistencyLayer::WorldKey k)
{
  return static_cast<int32_t>(static_cast<uint32_t>(k & 0xFFFFFFFFu));
}

GroundConsistencyLayer::GroundConsistencyLayer()
{
  costmap_ = nullptr;
}

void GroundConsistencyLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("GroundConsistencyLayer: node expired");
  }

  // Parameters
  declareParameter("ground_points_topic", rclcpp::ParameterValue(std::string("/ground_points")));
  declareParameter("nonground_points_topic", rclcpp::ParameterValue(std::string("/nonground_points")));
  declareParameter("tf_timeout", rclcpp::ParameterValue(0.1));
  declareParameter("ground_inc", rclcpp::ParameterValue(1.0));
  declareParameter("nonground_inc", rclcpp::ParameterValue(1.5));
  declareParameter("ground_decay", rclcpp::ParameterValue(0.92));
  declareParameter("nonground_decay", rclcpp::ParameterValue(0.90));
  declareParameter("nonground_occ_thresh", rclcpp::ParameterValue(2.0));
  declareParameter("nonground_prob_thresh", rclcpp::ParameterValue(0.750));
  declareParameter("max_score", rclcpp::ParameterValue(1000.0));
  declareParameter("min_clearance", rclcpp::ParameterValue(0.1)); // 10 cm: small obstacles max height
  declareParameter("robot_height", rclcpp::ParameterValue(1.2)); // 1.2 m: tunnel detection threshold
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true)); // Clear robot footprint polygon
  declareParameter("enable_kpi_logging", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".ground_points_topic", ground_topic_);
  node->get_parameter(name_ + ".nonground_points_topic", nonground_topic_);
  node->get_parameter(name_ + ".tf_timeout", tf_timeout_);
  node->get_parameter(name_ + ".ground_inc", ground_inc_);
  node->get_parameter(name_ + ".nonground_inc", nonground_inc_);
  node->get_parameter(name_ + ".ground_decay", ground_decay_);
  node->get_parameter(name_ + ".nonground_decay", nonground_decay_);
  node->get_parameter(name_ + ".nonground_occ_thresh", nonground_occ_thresh_);
  node->get_parameter(name_ + ".nonground_prob_thresh", nonground_prob_thresh_);
  node->get_parameter(name_ + ".max_score", max_score_);
  node->get_parameter(name_ + ".min_clearance", min_clearance_);
  node->get_parameter(name_ + ".robot_height", robot_height_);
  node->get_parameter(name_ + ".footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + ".enable_kpi_logging", kpi_enabled_);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  matchSize();

  RCLCPP_INFO(node->get_logger(), 
    "GroundConsistencyLayer initialized. global_frame=%s, "
    "ground_topic=%s, nonground_topic=%s, enable_kpi=%s",
    global_frame_.c_str(), ground_topic_.c_str(), nonground_topic_.c_str(),
    kpi_enabled_ ? "true" : "false");

  // Initialize KPI tracking
  if (kpi_enabled_) {
    std::string kpi_file = "/tmp/costmap_kpi_" + name_ + ".csv";
    kpi_tracker_ = std::make_unique<KPITracker>(kpi_file);
    RCLCPP_INFO(node->get_logger(),
      "KPI logging enabled. Output: %s", kpi_file.c_str());
  }

  enabled_ = true;
  current_ = true;
}

void GroundConsistencyLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"),
      "GroundConsistencyLayer::activate() - node expired");
    return;
  }

  auto qos = rclcpp::SensorDataQoS();
  ground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    ground_topic_, qos,
    std::bind(&GroundConsistencyLayer::groundCloudCallback, this, std::placeholders::_1));

  nonground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    nonground_topic_, qos,
    std::bind(&GroundConsistencyLayer::nongroundCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(),
    "GroundConsistencyLayer activated. Subscribed to %s and %s",
    ground_topic_.c_str(), nonground_topic_.c_str());
}

void GroundConsistencyLayer::deactivate()
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"),
      "GroundConsistencyLayer::deactivate() - node expired");
    return;
  }

  // Reset subscriptions (destroys them)
  ground_sub_ = nullptr;
  nonground_sub_ = nullptr;

  RCLCPP_INFO(node->get_logger(),
    "GroundConsistencyLayer deactivated. Unsubscribed from ground/nonground topics");
}


void GroundConsistencyLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());

  // Only clear per-frame accumulators; persistent scores use world-frame keys
  // and survive origin changes without shifting.
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & [key, cell] : cells_) {
    cell.ground_count_frame = 0;
    cell.nonground_count_frame = 0;
  }
}

void GroundConsistencyLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cells_.clear();
}

void GroundConsistencyLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  // Start cycle timer
  if (kpi_enabled_) {
    kpi_tracker_->startTimer();
  }

  double local_min_x = std::numeric_limits<double>::max();
  double local_min_y = std::numeric_limits<double>::max();
  double local_max_x = std::numeric_limits<double>::lowest();
  double local_max_y = std::numeric_limits<double>::lowest();

  {
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto & [key, cell] : cells_) {
      if (cell.ground_score <= 0.0f && cell.nonground_score <= 0.0f) continue;

      int32_t xi = unpackX(key);
      int32_t yi = unpackY(key);
      double wx = (xi + 0.5) * getResolution();
      double wy = (yi + 0.5) * getResolution();

      local_min_x = std::min(local_min_x, wx);
      local_max_x = std::max(local_max_x, wx);
      local_min_y = std::min(local_min_y, wy);
      local_max_y = std::max(local_max_y, wy);
    }
  }

  // If no data, just update footprint (don't update costmap data)
  if (local_min_x > local_max_x) {
    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    return;
  }

  *min_x = local_min_x;
  *min_y = local_min_y;
  *max_x = local_max_x;
  *max_y = local_max_y;

  // Transform robot footprint for clearing
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void GroundConsistencyLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  std::vector<geometry_msgs::msg::Point> footprint = getFootprint();

  if (footprint.empty()) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_ERROR(
        node->get_logger(),
        "GroundConsistencyLayer: No footprint configured! "
        "Please set 'footprint' parameter in costmap_2d config. "
        "Skipping footprint clearing.");
    }
    return;
  }

  // Transform robot footprint to current pose
  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, footprint, transformed_footprint_);

  // Update bounds to include footprint
  for (unsigned int i = 0; i < transformed_footprint_.size(); ++i) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

static bool lookupTF(
  const std::shared_ptr<tf2_ros::Buffer> & buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  double timeout_sec,
  geometry_msgs::msg::TransformStamped & out_tf)
{
  try {
    out_tf = buffer->lookupTransform(
      target_frame, source_frame, stamp,
      rclcpp::Duration::from_seconds(timeout_sec));
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void GroundConsistencyLayer::groundCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  // Validate input message
  if (!msg || msg->data.empty()) {
    RCLCPP_DEBUG(node->get_logger(),
      "GroundConsistencyLayer: Received empty ground point cloud");
    return;
  }

  if (msg->point_step == 0) {
    RCLCPP_WARN(node->get_logger(),
      "GroundConsistencyLayer: Invalid ground point cloud (point_step=0)");
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTF(tf_buffer_, global_frame_, msg->header.frame_id,
                msg->header.stamp, tf_timeout_, tf)) {
    RCLCPP_DEBUG(node->get_logger(),
      "GroundConsistencyLayer: TF lookup failed for ground cloud from %s",
      msg->header.frame_id.c_str());
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_global;
  try {
    tf2::doTransform(*msg, cloud_global, tf);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "GroundConsistencyLayer: Failed to transform ground cloud: %s", e.what());
    return;
  }

  // Validate required fields exist
  if (cloud_global.fields.empty()) {
    RCLCPP_WARN(node->get_logger(),
      "GroundConsistencyLayer: Ground point cloud has no fields");
    return;
  }

  std::unordered_map<WorldKey, uint32_t> local_counts;
  local_counts.reserve(4096);

  const double res = getResolution();

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_global, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_global, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_global, "z");

    // First pass: build local counts only
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      WorldKey key = worldKey(*iter_x, *iter_y, res);
      local_counts[key] += 1u;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "GroundConsistencyLayer: Failed to read ground cloud fields: %s", e.what());
    return;
  }

  // Lock before touching shared map
  std::lock_guard<std::mutex> lock(mutex_);

  // Update frame counts
  for (const auto& kv : local_counts) {
    cells_[kv.first].ground_count_frame += kv.second;
  }

  // Second pass: update height statistics safely
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x2(cloud_global, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y2(cloud_global, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z2(cloud_global, "z");

    for (; iter_x2 != iter_x2.end(); ++iter_x2, ++iter_y2, ++iter_z2) {
      WorldKey key = worldKey(*iter_x2, *iter_y2, res);
      auto & cell = cells_[key];
      cell.ground_height_sum += *iter_z2;
      cell.ground_height_count += 1u;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "GroundConsistencyLayer: Failed to process ground height stats: %s", e.what());
  }
}

void GroundConsistencyLayer::nongroundCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  // Validate input message
  if (!msg || msg->data.empty()) {
    RCLCPP_DEBUG(node->get_logger(),
      "GroundConsistencyLayer: Received empty non-ground point cloud");
    return;
  }

  if (msg->point_step == 0) {
    RCLCPP_WARN(node->get_logger(),
      "GroundConsistencyLayer: Invalid non-ground point cloud (point_step=0)");
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTF(tf_buffer_, global_frame_, msg->header.frame_id,
                msg->header.stamp, tf_timeout_, tf)) {
    RCLCPP_DEBUG(node->get_logger(),
      "GroundConsistencyLayer: TF lookup failed for non-ground cloud from %s",
      msg->header.frame_id.c_str());
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_global;
  try {
    tf2::doTransform(*msg, cloud_global, tf);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "GroundConsistencyLayer: Failed to transform non-ground cloud: %s", e.what());
    return;
  }

  // Validate required fields exist
  if (cloud_global.fields.empty()) {
    RCLCPP_WARN(node->get_logger(),
      "GroundConsistencyLayer: Non-ground point cloud has no fields");
    return;
  }

  std::unordered_map<WorldKey, uint32_t> local_counts;
  local_counts.reserve(4096);

  const double res = getResolution();

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_global, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_global, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_global, "z");

    // First pass: build local counts only
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      WorldKey key = worldKey(*iter_x, *iter_y, res);
      local_counts[key] += 1u;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "GroundConsistencyLayer: Failed to read non-ground cloud fields: %s", e.what());
    return;
  }

  // Lock before touching shared map
  std::lock_guard<std::mutex> lock(mutex_);

  // Update frame counts
  for (const auto& kv : local_counts) {
    cells_[kv.first].nonground_count_frame += kv.second;
  }

  // Second pass: update obstacle height stats safely
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x2(cloud_global, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y2(cloud_global, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z2(cloud_global, "z");

    for (; iter_x2 != iter_x2.end(); ++iter_x2, ++iter_y2, ++iter_z2) {
      WorldKey key = worldKey(*iter_x2, *iter_y2, res);
      auto & cell = cells_[key];
      cell.obstacle_min_height = std::min(cell.obstacle_min_height, static_cast<double>(*iter_z2));
      cell.obstacle_max_height = std::max(cell.obstacle_max_height, static_cast<double>(*iter_z2));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "GroundConsistencyLayer: Failed to process obstacle height stats: %s", e.what());
  }
}

void GroundConsistencyLayer::integrateFrameCountsIntoScores()
{
  for (auto & [key, cell] : cells_) {
    if (cell.ground_count_frame > 0) {
      cell.ground_score = static_cast<float>(
        std::min<double>(max_score_, cell.ground_score + cell.ground_count_frame * ground_inc_));
      cell.ground_count_frame = 0;
    }
    if (cell.nonground_count_frame > 0) {
      cell.nonground_score = static_cast<float>(
        std::min<double>(max_score_, cell.nonground_score + cell.nonground_count_frame * nonground_inc_));
      cell.nonground_count_frame = 0;
    }
  }
}

void GroundConsistencyLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);

  cells_updated_this_cycle_ = 0;
  cells_decayed_this_cycle_ = 0;

  // Capture frame counts before integration clears them
  uint32_t ground_points_this_cycle = 0;
  uint32_t nonground_points_this_cycle = 0;
  for (const auto & [key, cell] : cells_) {
    ground_points_this_cycle += cell.ground_count_frame;
    nonground_points_this_cycle += cell.nonground_count_frame;
  }

  integrateFrameCountsIntoScores();

  // Clear robot footprint evidence to prevent self-blocking
  if (footprint_clearing_enabled_ && !transformed_footprint_.empty()) {
    // Find bounds of footprint in world coordinates
    double fp_min_x = std::numeric_limits<double>::max();
    double fp_min_y = std::numeric_limits<double>::max();
    double fp_max_x = std::numeric_limits<double>::lowest();
    double fp_max_y = std::numeric_limits<double>::lowest();

    for (const auto & pt : transformed_footprint_) {
      fp_min_x = std::min(fp_min_x, pt.x);
      fp_min_y = std::min(fp_min_y, pt.y);
      fp_max_x = std::max(fp_max_x, pt.x);
      fp_max_y = std::max(fp_max_y, pt.y);
    }

    const double res = getResolution();
    const int32_t min_xi = static_cast<int32_t>(std::floor(fp_min_x / res));
    const int32_t min_yi = static_cast<int32_t>(std::floor(fp_min_y / res));
    const int32_t max_xi = static_cast<int32_t>(std::floor(fp_max_x / res));
    const int32_t max_yi = static_cast<int32_t>(std::floor(fp_max_y / res));

    // Remove evidence in footprint area
    for (int32_t xi = min_xi; xi <= max_xi; ++xi) {
      for (int32_t yi = min_yi; yi <= max_yi; ++yi) {
        cells_.erase(packXY(xi, yi));
      }
    }

    // Also mark the costmap cells as FREE_SPACE for immediate effect
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  const double res = getResolution();
  const float gd = static_cast<float>(ground_decay_);
  const float nd = static_cast<float>(nonground_decay_);

  // Optional hard obstacle thresholds (keep your existing params)
  const float oth = static_cast<float>(nonground_occ_thresh_);
  const float pth = static_cast<float>(nonground_prob_thresh_);
  const float eps = 1e-5f;

  // Rolling window bounds in world
  double min_wx, min_wy, max_wx, max_wy;
  master_grid.mapToWorld(min_i, min_j, min_wx, min_wy);
  master_grid.mapToWorld(max_i, max_j, max_wx, max_wy);

  if (min_wx > max_wx) std::swap(min_wx, max_wx);
  if (min_wy > max_wy) std::swap(min_wy, max_wy);

  // pad by one cell
  min_wx -= res; min_wy -= res;
  max_wx += res; max_wy += res;

  auto cellCenter = [&](int32_t xi, int32_t yi, double & wx, double & wy) {
    wx = (static_cast<double>(xi) + 0.5) * res;
    wy = (static_cast<double>(yi) + 0.5) * res;
  };

  auto inWindow = [&](double wx, double wy) -> bool {
    return wx >= min_wx && wx <= max_wx && wy >= min_wy && wy <= max_wy;
  };

  // Single pass: decay, compute costs, erase dead cells
  size_t total_ground_cells = 0;
  size_t total_nonground_cells = 0;

  for (auto it = cells_.begin(); it != cells_.end(); ) {
    auto & cell = it->second;
    int32_t xi = unpackX(it->first);
    int32_t yi = unpackY(it->first);
    double wx, wy;
    cellCenter(xi, yi, wx, wy);

    // Erase out-of-window cells
    if (!inWindow(wx, wy)) {
      it = cells_.erase(it);
      continue;
    }

    // Decay scores
    if (cell.ground_score > 0.0f) {
      cell.ground_score *= gd;
      cells_decayed_this_cycle_++;
    }
    if (cell.nonground_score > 0.0f) {
      cell.nonground_score *= nd;
      cells_decayed_this_cycle_++;
    }

    // Erase cells with no remaining evidence
    if (cell.ground_score < 1e-3f && cell.nonground_score < 1e-3f) {
      it = cells_.erase(it);
      continue;
    }

    // Clamp tiny scores to zero
    if (cell.ground_score < 1e-3f) cell.ground_score = 0.0f;
    if (cell.nonground_score < 1e-3f) cell.nonground_score = 0.0f;

    // Count for KPI
    if (cell.ground_score > 0.0f) total_ground_cells++;
    if (cell.nonground_score > 0.0f) total_nonground_cells++;

    // Compute cost
    float g = cell.ground_score;
    float ng = cell.nonground_score;
    const float denom = ng + g + eps;
    const float p_occ = ng / denom;
    uint8_t cost = static_cast<uint8_t>(std::clamp(p_occ * 252.0f, 0.0f, 252.0f));

    bool make_lethal = false;
    bool make_free = false;
    if (ng >= oth && p_occ > pth) {
      double ground_avg = 0.0;
      if (cell.ground_height_count > 0u) {
        ground_avg = cell.ground_height_sum / static_cast<double>(cell.ground_height_count);
      }

      double step_height = cell.obstacle_min_height - ground_avg;
      double obstacle_height = cell.obstacle_max_height - ground_avg;

      bool is_tunnel = step_height > robot_height_;
      bool is_small_obstacle = obstacle_height < min_clearance_;

      if (is_tunnel || is_small_obstacle) {
        make_free = true;
      } else {
        make_lethal = true;
      }
    }

    if (make_lethal) {
      cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    } else if (make_free) {
      cost = nav2_costmap_2d::FREE_SPACE;
    }

    // Write cost to master grid
    unsigned int mx, my;
    if (master_grid.worldToMap(wx, wy, mx, my)) {
      if ((int)mx >= min_i && (int)mx < max_i && (int)my >= min_j && (int)my < max_j) {
        master_grid.setCost(mx, my, cost);
        cells_updated_this_cycle_++;
      }
    }

    ++it;
  }

  // Record KPI metrics
  if (kpi_enabled_ && kpi_tracker_) {
    KPISnapshot snapshot;
    snapshot.timestamp = std::chrono::system_clock::now();
    snapshot.total_cycle_latency_ms = kpi_tracker_->getTotalTime();
    snapshot.cells_updated = cells_updated_this_cycle_;
    snapshot.cells_decayed = cells_decayed_this_cycle_;
    snapshot.total_ground_cells = total_ground_cells;
    snapshot.total_nonground_cells = total_nonground_cells;
    
    // Memory: single map with CellData struct + bucket overhead
    size_t entries = cells_.size();
    const size_t node_overhead = 56;  // libstdc++ hash node
    size_t mem_bytes = entries * (node_overhead + sizeof(WorldKey) + sizeof(CellData)) +
                       std::max(entries, size_t(16)) * 8;  // bucket array
    snapshot.memory_usage_mb = mem_bytes / (1024.0 * 1024.0);
    
    snapshot.ground_points_processed = ground_points_this_cycle;
    snapshot.nonground_points_processed = nonground_points_this_cycle;
    
    kpi_tracker_->recordSnapshot(snapshot);
  }
}



}  // namespace nav2_ground_consistency_costmap_plugin

PLUGINLIB_EXPORT_CLASS(
  nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer,
  nav2_costmap_2d::Layer)
