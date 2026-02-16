#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_ground_consistency_costmap_plugin
{

class GroundConsistencyLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  GroundConsistencyLayer();
  ~GroundConsistencyLayer() override = default;

  void onInitialize() override;

  void matchSize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  bool isClearable() override { return true; }

  using WorldKey = uint64_t;

private:
  // Pack two signed 32-bit grid coords (xi, yi) into one 64-bit key
  static inline WorldKey packXY(int32_t xi, int32_t yi)
  {
    return (static_cast<WorldKey>(static_cast<uint32_t>(xi)) << 32) |
          (static_cast<WorldKey>(static_cast<uint32_t>(yi)));
  }

  // Quantize a world position to a stable integer cell in global_frame_
  static inline WorldKey worldKey(double wx, double wy, double res)
  {
    // floor ensures stability across boundaries; use int32 for packing
    const int32_t xi = static_cast<int32_t>(std::floor(wx / res));
    const int32_t yi = static_cast<int32_t>(std::floor(wy / res));
    return packXY(xi, yi);
  }

  void groundCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void nongroundCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void integrateFrameCountsIntoScores();

  // Params
  std::string ground_topic_;
  std::string nonground_topic_;
  std::string global_frame_;
  double tf_timeout_{0.1};

  // Evidence model
  double ground_inc_{1.0};       // added per point (scaled by count)
  double nonground_inc_{2.0};    // added per point (scaled by count)
  double ground_decay_{0.90};    // multiplied each costmap update (0..1)
  double nonground_decay_{0.98}; // multiplied each costmap update (0..1)

  double nonground_occ_thresh_{3.0};   // nonground score must exceed to be OCCUPIED
  double nonground_prob_thresh_{0.7};   // nonground probability must exceed to be OCCUPIED

  double max_score_{1000.0};     // clamp to avoid overflow

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Per-frame counts in stable world grid (integrated once per update cycle)
  std::unordered_map<WorldKey, uint32_t> ground_counts_frame_;
  std::unordered_map<WorldKey, uint32_t> nonground_counts_frame_;

  // Persistent scores in stable world grid
  std::unordered_map<WorldKey, float> ground_score_world_;
  std::unordered_map<WorldKey, float> nonground_score_world_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_sub_;

  std::mutex mutex_;
};

}  // namespace nav2_ground_consistency_costmap_plugin
