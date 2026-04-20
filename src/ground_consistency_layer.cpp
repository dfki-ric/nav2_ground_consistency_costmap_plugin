#include "nav2_ground_consistency_costmap_plugin/ground_consistency_layer.hpp"

#include <algorithm>
#include <limits>
#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
  declareParameter("min_clearance", rclcpp::ParameterValue(0.1));
  declareParameter("robot_height", rclcpp::ParameterValue(1.2));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter("enable_kpi_logging", rclcpp::ParameterValue(false));
  declareParameter("max_data_range", rclcpp::ParameterValue(0.0));
  declareParameter("discretize_costs", rclcpp::ParameterValue(false));
  declareParameter("ground_neighbor_search_radius", rclcpp::ParameterValue(2));

  node->get_parameter(name_ + ".ground_points_topic", ground_topic_);
  node->get_parameter(name_ + ".nonground_points_topic", nonground_topic_);
  node->get_parameter(name_ + ".tf_timeout", tf_timeout_);

  auto getFloatParam = [&](const std::string & param, float & out) {
    double tmp;
    node->get_parameter(param, tmp);
    out = static_cast<float>(tmp);
  };

  getFloatParam(name_ + ".ground_inc", ground_inc_);
  getFloatParam(name_ + ".nonground_inc", nonground_inc_);
  getFloatParam(name_ + ".ground_decay", ground_decay_);
  getFloatParam(name_ + ".nonground_decay", nonground_decay_);
  getFloatParam(name_ + ".nonground_occ_thresh", nonground_occ_thresh_);
  getFloatParam(name_ + ".nonground_prob_thresh", nonground_prob_thresh_);

  node->get_parameter(name_ + ".max_score", max_score_);
  node->get_parameter(name_ + ".min_clearance", min_clearance_);
  node->get_parameter(name_ + ".robot_height", robot_height_);
  node->get_parameter(name_ + ".footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + ".enable_kpi_logging", kpi_enabled_);
  node->get_parameter(name_ + ".max_data_range", max_data_range_);
  node->get_parameter(name_ + ".discretize_costs", discretize_costs_);
  node->get_parameter(name_ + ".ground_neighbor_search_radius", ground_neighbor_search_radius_);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Read robot_base_frame from costmap config
  std::string robot_base_frame_param;
  node->get_parameter("robot_base_frame", robot_base_frame_param);
  robot_base_frame_ = robot_base_frame_param.empty() ? "base_link" : robot_base_frame_param;

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
  cells_.reserve(master->getSizeInCellsX() * master->getSizeInCellsY());
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

  std::lock_guard<std::mutex> lock(mutex_);

  cells_updated_this_cycle_ = 0;
  cells_decayed_this_cycle_ = 0;
  ground_points_this_cycle_ = 0;
  nonground_points_this_cycle_ = 0;

  // Transform footprint and update bounds to include it
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  // Clear footprint evidence
  if (footprint_clearing_enabled_ && !transformed_footprint_.empty()) {
    const double fp_res = getResolution();
    // Compute footprint bounding box in cell coordinates
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
    const int32_t min_xi = static_cast<int32_t>(std::floor(fp_min_x / fp_res));
    const int32_t min_yi = static_cast<int32_t>(std::floor(fp_min_y / fp_res));
    const int32_t max_xi = static_cast<int32_t>(std::floor(fp_max_x / fp_res));
    const int32_t max_yi = static_cast<int32_t>(std::floor(fp_max_y / fp_res));
    const size_t n = transformed_footprint_.size();

    for (int32_t xi = min_xi; xi <= max_xi; ++xi) {
      for (int32_t yi = min_yi; yi <= max_yi; ++yi) {
        double px = (static_cast<double>(xi) + 0.5) * fp_res;
        double py = (static_cast<double>(yi) + 0.5) * fp_res;

        // Point-in-polygon test (ray casting)
        bool inside = false;
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
          const auto & a = transformed_footprint_[i];
          const auto & b = transformed_footprint_[j];
          if ((a.y > py) != (b.y > py) &&
              px < (b.x - a.x) * (py - a.y) / (b.y - a.y) + a.x) {
            inside = !inside;
          }
        }
        if (inside) {
          cells_.erase(packXY(xi, yi));
        }
      }
    }
  }

  // Single pass: integrate frame counts, decay, compute costs, cull, compute bounds
  const float eps = 1e-5f;
  const double max_range_sq = max_data_range_ * max_data_range_;
  const bool use_range_limit = max_data_range_ > 0.0;
  const double res = getResolution();
  const bool have_new_data = have_new_data_;
  have_new_data_ = false;

  // Rolling window bounds for culling cells outside the costmap
  const nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  double win_origin_x = master->getOriginX();
  double win_origin_y = master->getOriginY();
  double win_max_x = win_origin_x + master->getSizeInMetersX();
  double win_max_y = win_origin_y + master->getSizeInMetersY();

  double local_min_x = std::numeric_limits<double>::max();
  double local_min_y = std::numeric_limits<double>::max();
  double local_max_x = std::numeric_limits<double>::lowest();
  double local_max_y = std::numeric_limits<double>::lowest();

  size_t total_ground_cells = 0;
  size_t total_nonground_cells = 0;

  // Cache robot Z for tier-3 fallback (once per cycle)
  double cached_robot_z = 0.0;
  bool have_robot_z = false;
  {
    geometry_msgs::msg::TransformStamped robot_tf;
    if (lookupTF(tf_buffer_, global_frame_, robot_base_frame_,
                 rclcpp::Time(0), tf_timeout_, robot_tf)) {
      cached_robot_z = robot_tf.transform.translation.z;
      have_robot_z = true;
    }
  }

  for (auto it = cells_.begin(); it != cells_.end(); ) {
    auto & cell = it->second;
    int32_t xi = unpackX(it->first);
    int32_t yi = unpackY(it->first);
    double wx = (static_cast<double>(xi) + 0.5) * res;
    double wy = (static_cast<double>(yi) + 0.5) * res;

    // Cull cells beyond max_data_range from robot
    if (use_range_limit) {
      double dx = wx - robot_x;
      double dy = wy - robot_y;
      if (dx * dx + dy * dy > max_range_sq) {
        it = cells_.erase(it);
        continue;
      }
    }

    // Cull cells outside the rolling costmap window
    if (wx < win_origin_x || wx > win_max_x || wy < win_origin_y || wy > win_max_y) {
      it = cells_.erase(it);
      continue;
    }

    // Integrate frame counts into scores (accumulate)
    if (cell.ground_count_frame > 0) {
      ground_points_this_cycle_ += cell.ground_count_frame;
      double new_score = cell.ground_count_frame * ground_inc_;
      cell.ground_score = static_cast<float>(
        std::min<double>(max_score_, cell.ground_score + new_score));
      cell.ground_count_frame = 0;
    }
    if (cell.nonground_count_frame > 0) {
      nonground_points_this_cycle_ += cell.nonground_count_frame;
      double new_score = cell.nonground_count_frame * nonground_inc_;
      cell.nonground_score = static_cast<float>(
        std::min<double>(max_score_, cell.nonground_score + new_score));
      // Refresh persistent heights with current frame observations
      cell.obstacle_min_height = cell.obstacle_min_height_frame;
      cell.obstacle_max_height = cell.obstacle_max_height_frame;
      cell.obstacle_min_height_frame = std::numeric_limits<double>::max();
      cell.obstacle_max_height_frame = std::numeric_limits<double>::lowest();
      cell.nonground_count_frame = 0;
    }

    // Decay scores only when sensor is actively providing data
    if (have_new_data) {
      bool decayed = false;
      if (cell.ground_score > 0.0f) {
        cell.ground_score *= ground_decay_;
        decayed = true;
      }
      if (cell.nonground_score > 0.0f) {
        cell.nonground_score *= nonground_decay_;
        decayed = true;
      }
      if (decayed) cells_decayed_this_cycle_++;

      if (cell.ground_score < 1e-3f && cell.nonground_score < 1e-3f) {
        it = cells_.erase(it);
        continue;
      }

      if (cell.ground_score < 1e-3f) cell.ground_score = 0.0f;
      if (cell.nonground_score < 1e-3f) cell.nonground_score = 0.0f;
    }

    // Count for KPI
    if (cell.ground_score > 0.0f) total_ground_cells++;
    if (cell.nonground_score > 0.0f) total_nonground_cells++;

    // Compute cost and store in cell
    float g = cell.ground_score;
    float ng = cell.nonground_score;
    const float denom = ng + g + eps;
    const float p_occ = ng / denom;
    uint8_t cost = static_cast<uint8_t>(std::clamp(p_occ * 252.0f, 0.0f, 252.0f));

    bool make_lethal = false;
    bool make_free = false;
    if (ng >= nonground_occ_thresh_ && p_occ > nonground_prob_thresh_) {
      double ground_avg = 0.0;
      bool ground_found = false;
      int tier_used = 0;  // 0=none, 1=local, 2=neighbor, 3=robot_z

      // Tier 1: Local ground data
      if (cell.ground_height_count > 0u) {
        ground_avg = cell.ground_height_sum / static_cast<double>(cell.ground_height_count);
        ground_found = true;
        tier_used = 1;
      }

      // Tier 2: Neighbor interpolation
      if (!ground_found) {
        double sum_z = 0.0;
        uint32_t count = 0;
        const int r = ground_neighbor_search_radius_;
        for (int dx = -r; dx <= r; ++dx) {
          for (int dy = -r; dy <= r; ++dy) {
            if (dx == 0 && dy == 0) continue;
            auto n_it = cells_.find(packXY(xi + dx, yi + dy));
            if (n_it != cells_.end() && n_it->second.ground_height_count > 0u) {
              sum_z += n_it->second.ground_height_sum /
                       static_cast<double>(n_it->second.ground_height_count);
              count++;
            }
          }
        }
        if (count > 0) {
          ground_avg = sum_z / count;
          ground_found = true;
          tier_used = 2;
        }
      }

      // Tier 3: Robot Z fallback
      if (!ground_found && have_robot_z) {
        ground_avg = cached_robot_z;
        ground_found = true;
        tier_used = 3;
      }

      // DEBUG: Log for gap cells with high obstacle evidence
      auto node = node_.lock();
      if (node && cell.ground_height_count == 0u) {
        const char* tier_names[] = {"NONE", "T1", "T2", "T3"};
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
          "GAP_CELL[%d,%d]: tier=%s found=%d obs_min=%.2f obs_max=%.2f ground_avg=%.2f robot_z=%.2f robot_h=%.2f step_h=%.2f",
          xi, yi, tier_names[tier_used], ground_found,
          cell.obstacle_min_height, cell.obstacle_max_height, ground_avg,
          cached_robot_z, robot_height_,
          ground_found ? (cell.obstacle_min_height - ground_avg) : -999.0);
      }

      // Apply height filtering or conservative fallback
      if (ground_found) {
        double step_height = cell.obstacle_min_height - ground_avg;
        double obstacle_height = cell.obstacle_max_height - ground_avg;

        if (step_height > robot_height_ || obstacle_height < min_clearance_) {
          make_free = true;
          
          // DEBUG: Check if FREE cells are being set for Tier 1 cells (with ground data)
          if (cell.ground_height_count > 0u) {
            if (node) {
              RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
                "TIER1_FREE: cell[%d,%d] obs_min=%.2f ground_avg=%.2f step_h=%.2f",
                xi, yi, cell.obstacle_min_height, ground_avg, step_height);
            }
          }
        } else {
          make_lethal = true;
          if (node) {
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
              "LETHAL_HEIGHT: cell[%d,%d] obs_min=%.2f obs_max=%.2f ground_avg=%.2f step_h=%.2f obs_h=%.2f robot_h=%.2f min_cl=%.2f g_count=%u",
              xi, yi, cell.obstacle_min_height, cell.obstacle_max_height, ground_avg,
              step_height, obstacle_height, robot_height_, min_clearance_, cell.ground_height_count);
          }
        }
      } else {
        // All tiers failed: conservative LETHAL
        make_lethal = true;
        if (node) {
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
            "LETHAL_NO_GROUND: cell[%d,%d] obs_min=%.2f obs_max=%.2f",
            xi, yi, cell.obstacle_min_height, cell.obstacle_max_height);
        }
      }
    }

    if (make_lethal) {
      cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    } else if (make_free) {
      cost = nav2_costmap_2d::FREE_SPACE;
    }

    // Discretize partial costs to FREE if enabled
    if (discretize_costs_ && 
        cost != nav2_costmap_2d::LETHAL_OBSTACLE && 
        cost != nav2_costmap_2d::FREE_SPACE) {
      cost = nav2_costmap_2d::FREE_SPACE;
    }

    cell.computed_cost = cost;

    // DEBUG: Check if FREE cells are actually being set for gap cells
    if (cell.ground_height_count == 0u && make_free) {
      auto node = node_.lock();
      if (node) {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
          "SETTING_FREE: cell[%d,%d] cost=%d (FREE_SPACE=%d LETHAL=%d)",
          xi, yi, (int)cost, (int)nav2_costmap_2d::FREE_SPACE, (int)nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }

    // Update bounds
    local_min_x = std::min(local_min_x, wx);
    local_max_x = std::max(local_max_x, wx);
    local_min_y = std::min(local_min_y, wy);
    local_max_y = std::max(local_max_y, wy);

    ++it;
  }

  // Store KPI counters for use in updateCosts
  total_ground_cells_ = total_ground_cells;
  total_nonground_cells_ = total_nonground_cells;

  if (local_min_x <= local_max_x) {
    *min_x = std::min(*min_x, local_min_x);
    *min_y = std::min(*min_y, local_min_y);
    *max_x = std::max(*max_x, local_max_x);
    *max_y = std::max(*max_y, local_max_y);
  }
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

  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, footprint, transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); ++i) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void GroundConsistencyLayer::groundCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

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

  geometry_msgs::msg::TransformStamped tf_stamped;
  if (!lookupTF(tf_buffer_, global_frame_, msg->header.frame_id,
                msg->header.stamp, tf_timeout_, tf_stamped)) {
    RCLCPP_DEBUG(node->get_logger(),
      "GroundConsistencyLayer: TF lookup failed for ground cloud from %s",
      msg->header.frame_id.c_str());
    return;
  }

  tf2::Transform transform;
  tf2::fromMsg(tf_stamped.transform, transform);
  const double res = getResolution();

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    std::lock_guard<std::mutex> lock(mutex_);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      tf2::Vector3 gp = transform(tf2::Vector3(*iter_x, *iter_y, *iter_z));

      WorldKey key = worldKey(gp.x(), gp.y(), res);
      auto & cell = cells_[key];
      cell.ground_count_frame += 1u;
      cell.ground_height_sum += gp.z();
      cell.ground_height_count += 1u;
    }
    have_new_data_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "GroundConsistencyLayer: Failed to process ground cloud: %s", e.what());
  }
}

void GroundConsistencyLayer::nongroundCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

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

  geometry_msgs::msg::TransformStamped tf_stamped;
  if (!lookupTF(tf_buffer_, global_frame_, msg->header.frame_id,
                msg->header.stamp, tf_timeout_, tf_stamped)) {
    RCLCPP_DEBUG(node->get_logger(),
      "GroundConsistencyLayer: TF lookup failed for non-ground cloud from %s",
      msg->header.frame_id.c_str());
    return;
  }

  tf2::Transform transform;
  tf2::fromMsg(tf_stamped.transform, transform);
  const double res = getResolution();

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    std::lock_guard<std::mutex> lock(mutex_);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      tf2::Vector3 gp = transform(tf2::Vector3(*iter_x, *iter_y, *iter_z));

      WorldKey key = worldKey(gp.x(), gp.y(), res);
      auto & cell = cells_[key];
      cell.nonground_count_frame += 1u;
      cell.obstacle_min_height_frame = std::min(cell.obstacle_min_height_frame, gp.z());
      cell.obstacle_max_height_frame = std::max(cell.obstacle_max_height_frame, gp.z());
    }
    have_new_data_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "GroundConsistencyLayer: Failed to process non-ground cloud: %s", e.what());
  }
}

void GroundConsistencyLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX();
  const double res = getResolution();

  // Write pre-computed costs to master grid by iterating over master grid cells
  // and looking up the corresponding world-key cell.  This avoids quantization
  // mismatch between the global world-key grid (floor(wx/res)) and the rolling-
  // window master grid ((wx-origin)/res) which can leave gaps as NO_INFORMATION.
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(static_cast<unsigned int>(i),
                             static_cast<unsigned int>(j), wx, wy);
      WorldKey key = worldKey(wx, wy, res);
      auto it = cells_.find(key);
      if (it != cells_.end()) {
        master_array[j * size_x + i] = it->second.computed_cost;
        cells_updated_this_cycle_++;
      }
    }
  }

  // Clear footprint on the master grid (after writing costs so it takes priority)
  if (footprint_clearing_enabled_ && !transformed_footprint_.empty()) {
    master_grid.setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  // Record KPI metrics
  if (kpi_enabled_ && kpi_tracker_) {
    KPISnapshot snapshot;
    snapshot.timestamp = std::chrono::system_clock::now();
    snapshot.total_cycle_latency_ms = kpi_tracker_->getTotalTime();
    snapshot.cells_updated = cells_updated_this_cycle_;
    snapshot.cells_decayed = cells_decayed_this_cycle_;
    snapshot.total_ground_cells = total_ground_cells_;
    snapshot.total_nonground_cells = total_nonground_cells_;
    
    size_t entries = cells_.size();
    const size_t node_overhead = 56;
    size_t mem_bytes = entries * (node_overhead + sizeof(WorldKey) + sizeof(CellData)) +
                       std::max(entries, size_t(16)) * 8;
    snapshot.memory_usage_mb = mem_bytes / (1024.0 * 1024.0);
    
    snapshot.ground_points_processed = ground_points_this_cycle_;
    snapshot.nonground_points_processed = nonground_points_this_cycle_;
    
    kpi_tracker_->recordSnapshot(snapshot);
  }
}



}  // namespace nav2_ground_consistency_costmap_plugin

PLUGINLIB_EXPORT_CLASS(
  nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer,
  nav2_costmap_2d::Layer)
