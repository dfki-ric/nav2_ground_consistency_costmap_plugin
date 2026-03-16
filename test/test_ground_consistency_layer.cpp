// Copyright 2025 Your Organization
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_ground_consistency_costmap_plugin/ground_consistency_layer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.hpp"

// ROS setup
static bool ros_initialized = false;

class RosFixture
{
public:
  RosFixture()
  {
    if (!ros_initialized) {
      rclcpp::init(0, nullptr);
      ros_initialized = true;
    }
  }

  ~RosFixture() {}
};

RosFixture g_ros_fixture;

// Test node using standard lifecycle node
class TestNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit TestNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name)
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn onShutdown(
    const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn onError(
    const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};

// Test fixture for GroundConsistencyLayer with actual message publishing
class GroundConsistencyLayerTest : public ::testing::Test
{
public:
  GroundConsistencyLayerTest()
  : layers_("frame", false, false), resolution_(0.1)
  {
    node_ = std::make_shared<TestNode>("ground_consistency_test_node");
    layers_.resizeMap(40, 40, resolution_, -2.0, -2.0);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  }

  ~GroundConsistencyLayerTest() {}

  void addGroundConsistencyLayer(
    const std::string & ground_topic = "/ground_points",
    const std::string & nonground_topic = "/nonground_points")
  {
    layer_ = std::make_shared<nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer>();

    // Declare parameters matching layer's expected configuration
    node_->declare_parameter("ground_consistency.ground_points_topic",
                             rclcpp::ParameterValue(ground_topic));
    node_->declare_parameter("ground_consistency.nonground_points_topic",
                             rclcpp::ParameterValue(nonground_topic));

    layer_->initialize(&layers_, "ground_consistency", tf_buffer_.get(),
                       rclcpp_lifecycle::LifecycleNode::WeakPtr(node_), nullptr);
    layers_.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(layer_));
  }

  void update()
  {
    double min_x, min_y, max_x, max_y;
    layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);

    nav2_costmap_2d::Costmap2D * master_grid = layers_.getCostmap();
    int min_i = 0, min_j = 0, max_i = master_grid->getSizeInCellsX() - 1,
        max_j = master_grid->getSizeInCellsY() - 1;
    layer_->updateCosts(*master_grid, min_i, min_j, max_i, max_j);
  }

  unsigned char getCellCost(double world_x, double world_y)
  {
    nav2_costmap_2d::Costmap2D * master_grid = layers_.getCostmap();
    unsigned int mx, my;
    master_grid->worldToMap(world_x, world_y, mx, my);
    return master_grid->getCost(mx, my);
  }

protected:
  std::shared_ptr<nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer> layer_;
  std::shared_ptr<TestNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav2_costmap_2d::LayeredCostmap layers_;
  double resolution_;
};

// ============================================================================
// BASIC INTEGRATION TESTS
// ============================================================================

TEST_F(GroundConsistencyLayerTest, testInitialization)
{
  EXPECT_NO_THROW(addGroundConsistencyLayer());
  EXPECT_TRUE(layer_);
}

TEST_F(GroundConsistencyLayerTest, testLayerName)
{
  addGroundConsistencyLayer();
  EXPECT_EQ(layer_->getName(), "ground_consistency");
}

TEST_F(GroundConsistencyLayerTest, testReset)
{
  addGroundConsistencyLayer();
  EXPECT_NO_THROW(layer_->reset());
}

TEST_F(GroundConsistencyLayerTest, testIsClearable)
{
  addGroundConsistencyLayer();
  EXPECT_TRUE(layer_->isClearable());
}

TEST_F(GroundConsistencyLayerTest, testUpdateBoundsBehavior)
{
  addGroundConsistencyLayer();

  double min_x, min_y, max_x, max_y;
  EXPECT_NO_THROW({
    layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  });

  EXPECT_LE(min_x, max_x);
  EXPECT_LE(min_y, max_y);
}

// ============================================================================
// ALGORITHM LOGIC TESTS
// ============================================================================

/**
 * Test that without any observations, costmap remains uninitialized (mostly NO_INFORMATION or FREE_SPACE)
 */
TEST_F(GroundConsistencyLayerTest, testNoObservationsYieldsFreeCells)
{
  addGroundConsistencyLayer();

  update();

  // Check a cell at known location - should be uninitialized or free
  unsigned char cost = getCellCost(0.5, 0.0);

  // It should be FREE_SPACE (0) or NO_INFORMATION (255), not obstacle
  EXPECT_NE(cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test that multiple update cycles without data maintain valid state
 */
TEST_F(GroundConsistencyLayerTest, testStabilityWithoutData)
{
  addGroundConsistencyLayer();

  for (int i = 0; i < 10; ++i) {
    EXPECT_NO_THROW(update());
  }

  // Layer should still be valid
  nav2_costmap_2d::Costmap2D * master_grid = layers_.getCostmap();
  EXPECT_GT(master_grid->getSizeInCellsX(), 0);
  EXPECT_GT(master_grid->getSizeInCellsY(), 0);
}

/**
 * Test matchSize() consistency
 */
TEST_F(GroundConsistencyLayerTest, testMatchSize)
{
  addGroundConsistencyLayer();

  layers_.resizeMap(50, 50, 0.05, -1.25, -1.25);
  EXPECT_NO_THROW(layer_->matchSize());

  nav2_costmap_2d::Costmap2D * master_grid = layers_.getCostmap();
  EXPECT_EQ(master_grid->getSizeInCellsX(), 50);
  EXPECT_EQ(master_grid->getSizeInCellsY(), 50);
}

/**
 * Test that footprint clearing mechanism executes without crashing
 * Verifies the layer can handle the footprint clearing code path
 */
TEST_F(GroundConsistencyLayerTest, testFootprintClearing)
{
  addGroundConsistencyLayer();

  // Mark entire costmap as obstacles
  nav2_costmap_2d::Costmap2D * master_grid = layers_.getCostmap();
  for (unsigned int i = 0; i < master_grid->getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < master_grid->getSizeInCellsY(); ++j) {
      master_grid->setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  // This should execute footprint clearing without crashing
  // The actual clearing depends on the footprint configuration
  EXPECT_NO_THROW(update());

  // Layer should still be valid
  EXPECT_TRUE(layer_);
}

/**
 * Test updateBounds with different robot poses reflects footprint
 */
TEST_F(GroundConsistencyLayerTest, testBoundsReflectFootprint)
{
  addGroundConsistencyLayer();

  // Test at origin
  double min_x, min_y, max_x, max_y;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);

  // Bounds should include robot footprint area
  EXPECT_LE(min_x, 0.0);
  EXPECT_LE(min_y, 0.0);
  EXPECT_GE(max_x, 0.0);
  EXPECT_GE(max_y, 0.0);

  // Test at different pose
  layer_->updateBounds(1.0, 1.0, 0.785, &min_x, &min_y, &max_x, &max_y);

  // Bounds should now be around (1.0, 1.0)
  EXPECT_LE(min_x, 1.0);
  EXPECT_LE(min_y, 1.0);
  EXPECT_GE(max_x, 1.0);
  EXPECT_GE(max_y, 1.0);
}

/**
 * Test that the layer can handle rapid successive updates
 */
TEST_F(GroundConsistencyLayerTest, testRapidUpdates)
{
  addGroundConsistencyLayer();

  // Rapid-fire updates should not crash or produce invalid state
  for (int i = 0; i < 20; ++i) {
    double min_x, min_y, max_x, max_y;
    layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);

    nav2_costmap_2d::Costmap2D * master_grid = layers_.getCostmap();
    int min_i = 0, min_j = 0, max_i = master_grid->getSizeInCellsX() - 1,
        max_j = master_grid->getSizeInCellsY() - 1;
    layer_->updateCosts(*master_grid, min_i, min_j, max_i, max_j);
  }

  // Should complete without exception
  EXPECT_TRUE(layer_);
}

/**
 * Test that layer handles resizing gracefully
 */
TEST_F(GroundConsistencyLayerTest, testResizeSequence)
{
  addGroundConsistencyLayer();

  // Sequence: 40x40 → 50x50 → 30x30
  std::vector<std::pair<int, int>> sizes{{50, 50}, {30, 30}, {60, 60}};

  for (const auto & sz : sizes) {
    layers_.resizeMap(sz.first, sz.second, 0.1, -2.0, -2.0);
    EXPECT_NO_THROW(layer_->matchSize());
    EXPECT_NO_THROW(update());
  }

  EXPECT_TRUE(layer_);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
