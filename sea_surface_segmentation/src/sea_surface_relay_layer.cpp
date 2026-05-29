
#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "sea_surface_segmentation/cost_mapping.hpp"

namespace sea_surface_layer
{

// Thin costmap layer that relays a peer SeaSurfaceLayer's published graded
// occupancy (running in a different costmap, typically local) into this costmap.
//
// Why a relay instead of a second SeaSurfaceLayer in the global costmap: the
// segmentation projection is the expensive step (N cameras × image-size
// iterations per frame); running it twice (once per costmap) would double the
// cost for no information gain since both costmaps end up with the same cells.
// The producer publishes once; this consumer copies. Mirrors the s57_grids /
// s57_layer producer/consumer pattern.
//
// The full graded gradient is relayed: each published cell maps through
// `occupancy_to_cost` to a soft cost (1..252) or LETHAL (254). Unobserved /
// no-opinion cells (-1, or any occupancy <= 0) map to "leave untouched", so the
// relay never clears another layer's marks — the global costmap's
// chart/inflation contributions stay authoritative; sea-surface only ADDS.
class SeaSurfaceRelayLayer: public nav2_costmap_2d::Layer
{
public:
  SeaSurfaceRelayLayer() {}
  ~SeaSurfaceRelayLayer() {}

  void onInitialize() override
  {
    auto node = node_.lock();
    declareParameter(
      "topic", rclcpp::ParameterValue(std::string("sea_surface/lethal_grid")));
    node->get_parameter(name_ + ".topic", topic_);

    global_frame_id_ = layered_costmap_->getGlobalFrameID();

    // Transient-local QoS matches the producer's publisher so a late-joining
    // global costmap still gets the most recent map (the producer publishes on
    // each updateBounds cycle ≈ 5–20 Hz; we cache the latest).
    grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&SeaSurfaceRelayLayer::gridCallback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(
      logger_,
      "SeaSurfaceRelayLayer subscribing to '" << topic_ << "' (expects frame '"
        << global_frame_id_ << "')");
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
  }

  // The producer owns clearing via decay; the relay never clears.
  bool isClearable() override { return false; }

  void updateBounds(
    double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg = latest_;
    }
    if (!msg) {
      return;
    }
    // Expand the master bounds to cover the published grid so the master's
    // updateCosts call reaches every cell the producer can mark — otherwise
    // an over-tight master bound would silently drop lethal cells from outside
    // it. The min/max merge is non-destructive: earlier layers' contributions
    // survive.
    const double ox = msg->info.origin.position.x;
    const double oy = msg->info.origin.position.y;
    const double sx = msg->info.width * msg->info.resolution;
    const double sy = msg->info.height * msg->info.resolution;
    *min_x = std::min(*min_x, ox);
    *min_y = std::min(*min_y, oy);
    *max_x = std::max(*max_x, ox + sx);
    *max_y = std::max(*max_y, oy + sy);
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg = latest_;
    }
    if (!msg) {
      return;
    }
    if (msg->info.resolution <= 0.0 || msg->info.width == 0 || msg->info.height == 0) {
      return;
    }

    const double ox = msg->info.origin.position.x;
    const double oy = msg->info.origin.position.y;
    const double inv_res = 1.0 / msg->info.resolution;
    const uint32_t mw = msg->info.width;
    const uint32_t mh = msg->info.height;

    // Walk the master cells in the requested bounds; for each, look up the
    // corresponding cell in the published grid. Looking the producer's cell up
    // by world position rather than direct index avoids requiring identical
    // origin / resolution between the two costmaps — the global costmap can
    // have its own pose / resolution and still pick up the producer's lethal
    // cells correctly.
    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        double wx, wy;
        master_grid.mapToWorld(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
        const double mx = (wx - ox) * inv_res;
        const double my = (wy - oy) * inv_res;
        if (mx < 0.0 || my < 0.0) {
          continue;
        }
        const auto x = static_cast<uint32_t>(mx);
        const auto y = static_cast<uint32_t>(my);
        if (x >= mw || y >= mh) {
          continue;
        }
        const int occ = msg->data[static_cast<size_t>(y) * mw + x];
        const int c = sea_surface_segmentation::occupancy_to_cost(occ);
        if (c >= 0) {
          master_grid.setCost(
            static_cast<unsigned int>(i), static_cast<unsigned int>(j),
            static_cast<unsigned char>(c));
        }
      }
    }
    // Considered current once a grid has been received and consumed.
    current_ = true;
  }

  void matchSize() override {}  // no internal buffer to resize

private:
  void gridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->header.frame_id != global_frame_id_) {
      if (auto node = node_.lock()) {
        auto clock = node->get_clock();
        RCLCPP_WARN_THROTTLE(
          logger_, *clock, 5000,
          "SeaSurfaceRelayLayer received grid in frame '%s' but expects '%s'; "
          "stamping LETHAL cells at the wrong world positions is a safety risk — "
          "dropping the message",
          msg->header.frame_id.c_str(), global_frame_id_.c_str());
      }
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = msg;
  }

  std::string topic_;
  std::string global_frame_id_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_;
  std::mutex mutex_;
};

}  // namespace sea_surface_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sea_surface_layer::SeaSurfaceRelayLayer, nav2_costmap_2d::Layer)
