
#include <gtest/gtest.h>

#include <memory>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

// Verify the SeaSurfaceLayer + SeaSurfaceRelayLayer plugins load via pluginlib
// from the package's costmap_plugins.xml descriptor — catches plugin
// registration issues (wrong class name in the export macro, missing
// `<class>` entry, broken shared library).
TEST(SeaSurfaceLayerPlugin, LoadsViaPluginlib)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");

  std::shared_ptr<nav2_costmap_2d::Layer> producer;
  ASSERT_NO_THROW(
    producer = loader.createSharedInstance("sea_surface_layer::SeaSurfaceLayer"));
  EXPECT_NE(producer, nullptr);

  std::shared_ptr<nav2_costmap_2d::Layer> relay;
  ASSERT_NO_THROW(
    relay = loader.createSharedInstance("sea_surface_layer::SeaSurfaceRelayLayer"));
  EXPECT_NE(relay, nullptr);
}

// Initialize the SeaSurfaceLayer against a real LayeredCostmap + LifecycleNode.
// This exercises `onInitialize` — the path that, before this regression test
// existed, hid a declaration-order bug from the unit-test set: registering
// `add_on_set_parameters_callback` *before* `declareParameter("published_topic")`
// caused the callback's configure-time rejection to fire during
// `declare_parameter`'s initial-value validation, throwing
// `InvalidParameterValueException` and aborting `onInitialize`. The layer
// would fail to load at deployment time but every GTest passed because none
// exercised the pluginlib initialization path.
//
// Keep this test guard-rail tight: a regression in declaration order, in any
// `declareParameter` validation in `onInitialize`, or in the param callback's
// `is_configure_time` matcher will all surface here as an `onInitialize` throw.
class LayerInitializeFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_sea_surface_layer");
    // The Layer constructor reads `layered_costmap_->getCostmap()`; size it
    // non-zero so the layer's `matchSize()` builds a valid `OccupancyBuffer`.
    parent_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
    parent_->resizeMap(/*size_x_cells=*/10, /*size_y_cells=*/10,
      /*resolution=*/1.0, /*origin_x=*/0.0, /*origin_y=*/0.0);
    tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_costmap_2d::Layer>>(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  }

  void TearDown() override
  {
    loader_.reset();
    tf_.reset();
    parent_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> parent_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_costmap_2d::Layer>> loader_;
};

TEST_F(LayerInitializeFixture, SeaSurfaceLayerInitializesWithoutThrow)
{
  auto plugin = loader_->createSharedInstance("sea_surface_layer::SeaSurfaceLayer");
  ASSERT_NE(plugin, nullptr);
  ASSERT_NO_THROW(
    plugin->initialize(parent_.get(), "sea_surface_layer", tf_.get(),
      node_, callback_group_));
}

TEST_F(LayerInitializeFixture, SeaSurfaceRelayLayerInitializesWithoutThrow)
{
  auto plugin = loader_->createSharedInstance("sea_surface_layer::SeaSurfaceRelayLayer");
  ASSERT_NE(plugin, nullptr);
  ASSERT_NO_THROW(
    plugin->initialize(parent_.get(), "sea_surface_relay", tf_.get(),
      node_, callback_group_));
}
