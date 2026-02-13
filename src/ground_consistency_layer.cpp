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

#include "nav2_costmap_2d/cost_values.hpp"

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
  declareParameter("ground_free_thresh", rclcpp::ParameterValue(5.0));
  declareParameter("nonground_occ_thresh", rclcpp::ParameterValue(3.0));
  declareParameter("max_score", rclcpp::ParameterValue(1000.0));
  declareParameter("unknown_as_occupied", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".ground_points_topic", ground_topic_);
  node->get_parameter(name_ + ".nonground_points_topic", nonground_topic_);
  node->get_parameter(name_ + ".tf_timeout", tf_timeout_);

  node->get_parameter(name_ + ".ground_inc", ground_inc_);
  node->get_parameter(name_ + ".nonground_inc", nonground_inc_);
  node->get_parameter(name_ + ".ground_decay", ground_decay_);
  node->get_parameter(name_ + ".nonground_decay", nonground_decay_);
  node->get_parameter(name_ + ".ground_free_thresh", ground_free_thresh_);
  node->get_parameter(name_ + ".nonground_occ_thresh", nonground_occ_thresh_);
  node->get_parameter(name_ + ".nonground_prob_thresh", nonground_prob_thresh_);
  node->get_parameter(name_ + ".max_score", max_score_);
  node->get_parameter(name_ + ".unknown_as_occupied", unknown_as_occupied_);

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
  double, double, double,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  *min_x = std::numeric_limits<double>::lowest();
  *min_y = std::numeric_limits<double>::lowest();
  *max_x = std::numeric_limits<double>::max();
  *max_y = std::numeric_limits<double>::max();
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
  if (!lookupTF(tf_buffer_, global_frame_, msg->header.frame_id, msg->header.stamp, tf_timeout_, tf)) {
    RCLCPP_WARN(
      node->get_logger(),
      "[GCL][ground] TF failed %s -> %s",
      msg->header.frame_id.c_str(), global_frame_.c_str());
    return;
  }

  std::unordered_map<WorldKey, uint32_t> local_counts;
  local_counts.reserve(4096);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    geometry_msgs::msg::PointStamped ps, pg;
    ps.header = msg->header;
    ps.point.x = *iter_x;
    ps.point.y = *iter_y;
    ps.point.z = *iter_z;

    tf2::doTransform(ps, pg, tf);

    const double res = getResolution();
    local_counts[worldKey(pg.point.x, pg.point.y, res)] += 1u;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & kv : local_counts) {
    ground_counts_frame_[kv.first] += kv.second;
  }
}

void GroundConsistencyLayer::nongroundCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTF(tf_buffer_, global_frame_, msg->header.frame_id, msg->header.stamp, tf_timeout_, tf)) {
    RCLCPP_WARN(
      node->get_logger(),
      "[GCL][nonground] TF failed %s -> %s",
      msg->header.frame_id.c_str(), global_frame_.c_str());
    return;
  }

  std::unordered_map<WorldKey, uint32_t> local_counts;
  local_counts.reserve(4096);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    geometry_msgs::msg::PointStamped ps, pg;
    ps.header = msg->header;
    ps.point.x = *iter_x;
    ps.point.y = *iter_y;
    ps.point.z = *iter_z;

    tf2::doTransform(ps, pg, tf);

    const double res = getResolution();
    local_counts[worldKey(pg.point.x, pg.point.y, res)] += 1u;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & kv : local_counts) {
    nonground_counts_frame_[kv.first] += kv.second;
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

  // --- 1) Decay both maps (within window) + erase tiny entries
  for (auto it = ground_score_world_.begin(); it != ground_score_world_.end(); ) {
    double wx, wy;
    cellCenter(unpackX(it->first), unpackY(it->first), wx, wy);
    if (wx < min_wx || wx > max_wx || wy < min_wy || wy > max_wy) {
      it = ground_score_world_.erase(it);
      continue; 
    }
    it->second *= gd;
    if (it->second < 1e-3f) it = ground_score_world_.erase(it);
    else ++it;
  }

  for (auto it = nonground_score_world_.begin(); it != nonground_score_world_.end(); ) {
    double wx, wy;
    cellCenter(unpackX(it->first), unpackY(it->first), wx, wy);
    if (wx < min_wx || wx > max_wx || wy < min_wy || wy > max_wy) {
      it = nonground_score_world_.erase(it);
      continue; 
    }

    it->second *= nd;
    if (it->second < 1e-3f) it = nonground_score_world_.erase(it);
    else ++it;
  }

  // --- 2) Iterate union of keys in-window and compute p_occ
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

    const float denom = ng + g + eps;
    const float p_occ = ng / denom;                  // 0..1
    uint8_t cost = static_cast<uint8_t>(std::clamp(p_occ * 252.0f, 0.0f, 252.0f));

    // Optional: hard lethal if very confident obstacle
    if (ng >= oth && p_occ > pth) {
      cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    }

    costs[k] = cost;
  };

  for (const auto & kv : ground_score_world_)   add_key(kv.first);
  for (const auto & kv : nonground_score_world_) add_key(kv.first);

  // --- 3) Write costs into master grid
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
