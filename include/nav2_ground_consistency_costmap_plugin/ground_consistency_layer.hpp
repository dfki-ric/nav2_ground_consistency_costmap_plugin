#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_ground_consistency_costmap_plugin/kpi_tracker.hpp"

namespace nav2_ground_consistency_costmap_plugin
{

class GroundConsistencyLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  GroundConsistencyLayer();
  ~GroundConsistencyLayer() override = default;

  void onInitialize() override;

  void activate() override;

  void deactivate() override;

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

  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);

  // Params
  std::string ground_topic_;
  std::string nonground_topic_;
  std::string global_frame_;
  double tf_timeout_{0.1};

  // Evidence model
  double ground_inc_{1.0};       // added per point (scaled by count)
  double nonground_inc_{1.5};    // added per point (scaled by count)
  double ground_decay_{0.92};    // multiplied each costmap update (0..1)
  double nonground_decay_{0.90}; // multiplied each costmap update (0..1)

  double nonground_occ_thresh_{2.0};    // nonground score must exceed to be OCCUPIED
  double nonground_prob_thresh_{0.750}; // nonground probability must exceed to be OCCUPIED

  double max_score_{1000.0};     // clamp to avoid overflow

  double min_clearance_{0.1};    // 10 cm: small obstacles max height
  double robot_height_{1.2};     // 1.2 m: tunnel detection threshold
  double max_data_range_{0.0};   // max distance from robot to retain data (0 = disabled)

  // Footprint clearing
  bool footprint_clearing_enabled_{true};
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // All per-cell data in a single struct (one hash lookup, one allocation)
  struct CellData {
    float ground_score{0.0f};
    float nonground_score{0.0f};
    uint32_t ground_height_count{0};
    double ground_height_sum{0.0};
    double obstacle_min_height{std::numeric_limits<double>::max()};
    double obstacle_max_height{std::numeric_limits<double>::lowest()};
    uint32_t ground_count_frame{0};
    uint32_t nonground_count_frame{0};
    uint8_t computed_cost{0};
  };

  std::unordered_map<WorldKey, CellData> cells_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_sub_;

  std::mutex mutex_;
  bool have_new_data_{false};

  // KPI Tracking
  std::unique_ptr<KPITracker> kpi_tracker_;
  bool kpi_enabled_{false};
  uint32_t cells_updated_this_cycle_{0};
  uint32_t cells_decayed_this_cycle_{0};
  uint32_t ground_points_this_cycle_{0};
  uint32_t nonground_points_this_cycle_{0};
  size_t total_ground_cells_{0};
  size_t total_nonground_cells_{0};
};

}  // namespace nav2_ground_consistency_costmap_plugin
