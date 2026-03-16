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
  declareParameter("nonground_inc", rclcpp::ParameterValue(2.0));
  declareParameter("ground_decay", rclcpp::ParameterValue(0.90));
  declareParameter("nonground_decay", rclcpp::ParameterValue(0.98));
  declareParameter("nonground_occ_thresh", rclcpp::ParameterValue(3.0));
  declareParameter("nonground_prob_thresh", rclcpp::ParameterValue(0.7));
  declareParameter("max_score", rclcpp::ParameterValue(1000.0));
  declareParameter("min_clearance", rclcpp::ParameterValue(0.10)); // meters
  declareParameter("robot_height", rclcpp::ParameterValue(1.2));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));

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

  global_frame_ = layered_costmap_->getGlobalFrameID();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::SensorDataQoS();
  ground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    ground_topic_, qos,
    std::bind(&GroundConsistencyLayer::groundCloudCallback, this, std::placeholders::_1));

  nonground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    nonground_topic_, qos,
    std::bind(&GroundConsistencyLayer::nongroundCloudCallback, this, std::placeholders::_1));

  matchSize();


  RCLCPP_WARN(node->get_logger(), "global_frame_=%s",
  global_frame_.c_str());

  enabled_ = true;
  current_ = true;
}

void GroundConsistencyLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());

  std::lock_guard<std::mutex> lock(mutex_);
  ground_counts_frame_.clear();
  nonground_counts_frame_.clear();
  ground_score_world_.clear();
  nonground_score_world_.clear();

  ground_height_count_world_.clear();
  ground_height_sum_world_.clear();
  obstacle_min_height_world_.clear();
  obstacle_max_height_world_.clear();
}

void GroundConsistencyLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ground_counts_frame_.clear();
  nonground_counts_frame_.clear();
  ground_score_world_.clear();
  nonground_score_world_.clear();
}

void GroundConsistencyLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  *min_x = std::numeric_limits<double>::lowest();
  *min_y = std::numeric_limits<double>::lowest();
  *max_x = std::numeric_limits<double>::max();
  *max_y = std::numeric_limits<double>::max();

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

  // Get robot footprint from costmap
  std::vector<geometry_msgs::msg::Point> footprint = getFootprint();

  if (footprint.empty()) {
    // If no footprint is configured, use a circular default (0.5m radius)
    const double default_radius = 0.5;
    for (int angle = 0; angle < 16; ++angle) {
      geometry_msgs::msg::Point pt;
      pt.x = default_radius * std::cos(2.0 * M_PI * angle / 16.0);
      pt.y = default_radius * std::sin(2.0 * M_PI * angle / 16.0);
      pt.z = 0.0;
      footprint.push_back(pt);
    }
  }

  // Transform robot footprint to current pose
  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, footprint, transformed_footprint_);

  // Update bounds to include footprint
  for (unsigned int i = 0; i < transformed_footprint_.size(); ++i) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

static inline int32_t unpackX(nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer::WorldKey k)
{
  return static_cast<int32_t>(static_cast<uint32_t>(k >> 32));
}

static inline int32_t unpackY(nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer::WorldKey k)
{
  return static_cast<int32_t>(static_cast<uint32_t>(k & 0xFFFFFFFFu));
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

  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTF(tf_buffer_, global_frame_, msg->header.frame_id,
                msg->header.stamp, tf_timeout_, tf)) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_global;
  tf2::doTransform(*msg, cloud_global, tf);

  std::unordered_map<WorldKey, uint32_t> local_counts;
  local_counts.reserve(4096);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_global, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_global, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_global, "z");

  const double res = getResolution();

  // First pass: build local counts only
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    WorldKey key = worldKey(*iter_x, *iter_y, res);
    local_counts[key] += 1u;
  }

  // Lock before touching shared maps
  std::lock_guard<std::mutex> lock(mutex_);

  // Update frame counts
  for (const auto& kv : local_counts) {
    ground_counts_frame_[kv.first] += kv.second;
  }

  // Second pass: update height statistics safely
  sensor_msgs::PointCloud2ConstIterator<float> iter_x2(cloud_global, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y2(cloud_global, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z2(cloud_global, "z");

  for (; iter_x2 != iter_x2.end(); ++iter_x2, ++iter_y2, ++iter_z2) {
    WorldKey key = worldKey(*iter_x2, *iter_y2, res);
    ground_height_sum_world_[key] += *iter_z2;
    ground_height_count_world_[key] += 1u;
  }
}

void GroundConsistencyLayer::nongroundCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTF(tf_buffer_, global_frame_, msg->header.frame_id,
                msg->header.stamp, tf_timeout_, tf)) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_global;
  tf2::doTransform(*msg, cloud_global, tf);

  std::unordered_map<WorldKey, uint32_t> local_counts;
  local_counts.reserve(4096);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_global, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_global, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_global, "z");

  const double res = getResolution();

  // First pass: build local counts only
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    WorldKey key = worldKey(*iter_x, *iter_y, res);
    local_counts[key] += 1u;
  }

  // Lock before touching shared maps
  std::lock_guard<std::mutex> lock(mutex_);

  // Update frame counts
  for (const auto& kv : local_counts) {
    nonground_counts_frame_[kv.first] += kv.second;
  }

  // Second pass: update obstacle height stats safely
  sensor_msgs::PointCloud2ConstIterator<float> iter_x2(cloud_global, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y2(cloud_global, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z2(cloud_global, "z");

  for (; iter_x2 != iter_x2.end(); ++iter_x2, ++iter_y2, ++iter_z2) {
    WorldKey key = worldKey(*iter_x2, *iter_y2, res);

    auto min_it = obstacle_min_height_world_.find(key);
    if (min_it == obstacle_min_height_world_.end()) {
      obstacle_min_height_world_[key] = *iter_z2;
      obstacle_max_height_world_[key] = *iter_z2;
    } else {
      obstacle_min_height_world_[key] =
        std::min(obstacle_min_height_world_[key], static_cast<double>(*iter_z2));
      obstacle_max_height_world_[key] =
        std::max(obstacle_max_height_world_[key], static_cast<double>(*iter_z2));
    }
  }
}

void GroundConsistencyLayer::integrateFrameCountsIntoScores()
{
  for (const auto & kv : ground_counts_frame_) {
    float & s = ground_score_world_[kv.first];
    s = static_cast<float>(std::min<double>(max_score_, s + kv.second * ground_inc_));
  }

  for (const auto & kv : nonground_counts_frame_) {
    float & s = nonground_score_world_[kv.first];
    s = static_cast<float>(std::min<double>(max_score_, s + kv.second * nonground_inc_));
  }

  ground_counts_frame_.clear();
  nonground_counts_frame_.clear();
}

void GroundConsistencyLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);

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

    // Remove evidence scores in footprint area
    for (int32_t xi = min_xi; xi <= max_xi; ++xi) {
      for (int32_t yi = min_yi; yi <= max_yi; ++yi) {
        WorldKey key = packXY(xi, yi);
        ground_score_world_.erase(key);
        nonground_score_world_.erase(key);
        ground_height_count_world_.erase(key);
        ground_height_sum_world_.erase(key);
        obstacle_min_height_world_.erase(key);
        obstacle_max_height_world_.erase(key);
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

  // Decay both maps (within window) + erase tiny entries
  for (auto it = ground_score_world_.begin(); it != ground_score_world_.end(); ) {
    double wx, wy;
    cellCenter(unpackX(it->first), unpackY(it->first), wx, wy);
    if (wx < min_wx || wx > max_wx || wy < min_wy || wy > max_wy) {
      it = ground_score_world_.erase(it);
      continue; 
    }
    it->second *= gd;
    if (it->second < 1e-3f)
    {
        WorldKey k = it->first;

        // erase associated terrain data
        ground_height_sum_world_.erase(k);
        ground_height_count_world_.erase(k);
        obstacle_min_height_world_.erase(k);
        obstacle_max_height_world_.erase(k);

        it = ground_score_world_.erase(it);
    }
    else
    {
        ++it;
    }
  }

  for (auto it = nonground_score_world_.begin(); it != nonground_score_world_.end(); ) {
    double wx, wy;
    cellCenter(unpackX(it->first), unpackY(it->first), wx, wy);
    if (wx < min_wx || wx > max_wx || wy < min_wy || wy > max_wy) {
      it = nonground_score_world_.erase(it);
      continue; 
    }

    it->second *= nd;
    if (it->second < 1e-3f)
    {
        WorldKey k = it->first;

        // erase associated terrain data
        ground_height_sum_world_.erase(k);
        ground_height_count_world_.erase(k);
        obstacle_min_height_world_.erase(k);
        obstacle_max_height_world_.erase(k);

        it = nonground_score_world_.erase(it);
    }
    else
    {
        ++it;
    }
  }

  // Iterate union of keys in-window and compute p_occ
  // We'll build a temporary set of keys (cheap enough for your window sizes).
  std::unordered_map<WorldKey, uint8_t> costs;
  costs.reserve(4096);

  auto add_key = [&](WorldKey k) {
    const int32_t xi = unpackX(k);
    const int32_t yi = unpackY(k);

    double wx, wy;
    cellCenter(xi, yi, wx, wy);
    if (wx < min_wx || wx > max_wx || wy < min_wy || wy > max_wy) return;

    float g = 0.0f;
    float ng = 0.0f;

    if (auto itg = ground_score_world_.find(k); itg != ground_score_world_.end()) g = itg->second;
    if (auto itn = nonground_score_world_.find(k); itn != nonground_score_world_.end()) ng = itn->second;

    uint8_t cost{0};

    const float denom = ng + g + eps;
    const float p_occ = ng / denom;
    cost = static_cast<uint8_t>(std::clamp(p_occ * 252.0f, 0.0f, 252.0f));

    bool make_lethal = false;
    bool make_free = false;
    if (ng >= oth && p_occ > pth)
    {
        double ground_avg = 0.0;
        auto it_sum = ground_height_sum_world_.find(k);
        auto it_cnt = ground_height_count_world_.find(k);
        if (it_sum != ground_height_sum_world_.end() &&
            it_cnt != ground_height_count_world_.end() &&
            it_cnt->second > 0u)
        {
          ground_avg = it_sum->second / static_cast<double>(it_cnt->second);
        }
        double obs_min = 0.0;
        double obs_max = 0.0;
        if (obstacle_min_height_world_.count(k))
        {
            obs_min = obstacle_min_height_world_[k];
            obs_max = obstacle_max_height_world_[k];
        }

        double step_height = obs_min - ground_avg;
        double obstacle_height = obs_max - ground_avg;

        bool is_tunnel = step_height > robot_height_;
        bool is_small_obstacle = obstacle_height < min_clearance_;

        if (is_tunnel || is_small_obstacle)
        {
          make_free = true;   // explicit override to free
        }
        else
        {
          make_lethal = true; // confirmed blocking
        }
    }

    // Apply overrides
    if (make_lethal)
    {
        cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    }
    else if (make_free)
    {
        cost = nav2_costmap_2d::FREE_SPACE;
    }

    costs[k] = cost;
  };

  for (const auto & kv : ground_score_world_)   add_key(kv.first);
  for (const auto & kv : nonground_score_world_) add_key(kv.first);

  //Write costs into master grid
  for (const auto & kv : costs) {
    const int32_t xi = unpackX(kv.first);
    const int32_t yi = unpackY(kv.first);

    double wx, wy;
    cellCenter(xi, yi, wx, wy);

    unsigned int mx, my;
    if (!master_grid.worldToMap(wx, wy, mx, my)) continue;
    if ((int)mx < min_i || (int)mx >= max_i || (int)my < min_j || (int)my >= max_j) continue;

    master_grid.setCost(mx, my, kv.second);
  }
}



}  // namespace nav2_ground_consistency_costmap_plugin

PLUGINLIB_EXPORT_CLASS(
  nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer,
  nav2_costmap_2d::Layer)
