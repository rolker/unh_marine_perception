
#include <algorithm>
#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "occupancy_buffer.hpp"
#include "segments_projection.hpp"

namespace sea_surface_layer
{

// Costmap layer that fuses camera segmentation into a persistent, decaying
// log-odds occupancy buffer (see occupancy_buffer.hpp). Each segmentation frame
// contributes ground-plane observations via project_observations (waterline
// contact → obstacle, water → free, occluded body → skipped); the buffer
// remembers obstacles through gaps and forgets them over time. Phase 1+3 of
// #19; still single-source (multi-camera fusion is phase 4).
class SeaSurfaceLayer: public nav2_costmap_2d::Layer
{
public:
  SeaSurfaceLayer() {}
  ~SeaSurfaceLayer() {}

  void onInitialize() override
  {
    auto node = node_.lock();

    declareParameter("maximum_range", rclcpp::ParameterValue(maximum_range_));
    node->get_parameter(name_ + ".maximum_range", maximum_range_);

    // Log-odds occupancy parameters (runtime-tunable wiring is phase 6).
    declareParameter("hit_log_odds", rclcpp::ParameterValue(params_.hit_log_odds));
    node->get_parameter(name_ + ".hit_log_odds", params_.hit_log_odds);
    declareParameter("miss_log_odds", rclcpp::ParameterValue(params_.miss_log_odds));
    node->get_parameter(name_ + ".miss_log_odds", params_.miss_log_odds);
    declareParameter("clamp", rclcpp::ParameterValue(params_.clamp));
    node->get_parameter(name_ + ".clamp", params_.clamp);
    declareParameter("lethal_threshold", rclcpp::ParameterValue(params_.lethal_threshold));
    node->get_parameter(name_ + ".lethal_threshold", params_.lethal_threshold);
    declareParameter("decay_half_life_s", rclcpp::ParameterValue(params_.decay_half_life_s));
    node->get_parameter(name_ + ".decay_half_life_s", params_.decay_half_life_s);

    std::string why;
    if (!sea_surface_segmentation::OccupancyBuffer::validate(params_, why)) {
      RCLCPP_WARN_STREAM(
        logger_, "Invalid occupancy params (" << why << "); using defaults.");
      params_ = sea_surface_segmentation::OccupancyParams{};
    }

    declareParameter("segmentation_topic", rclcpp::ParameterValue("segmentation"));
    std::string segmentation_topic;
    node->get_parameter(name_ + ".segmentation_topic", segmentation_topic);
    declareParameter("camera_info_topic", rclcpp::ParameterValue("camera_info"));
    std::string camera_info_topic;
    node->get_parameter(name_ + ".camera_info_topic", camera_info_topic);

    global_frame_id_ = layered_costmap_->getGlobalFrameID();

    // Build the buffer before wiring subscribers so a queued segmentation
    // message can't dispatch segmentsCallback against a null buffer_.
    matchSize();

    segments_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
      segmentation_topic, rclcpp::SensorDataQoS(),
      std::bind(&SeaSurfaceLayer::segmentsCallback, this, std::placeholders::_1));

    camera_info_subscriber_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      std::bind(&SeaSurfaceLayer::cameraInfoCallback, this, std::placeholders::_1));
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (buffer_) {
      buffer_->clear();
    }
  }

  // The layer clears itself via decay + water-miss observations; nav2's
  // clear-costmap behaviors don't need to (and shouldn't) wipe it.
  bool isClearable() override { return false; }

  void updateBounds(
    double robot_x, double robot_y, double /*robot_yaw*/,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    // Expand (don't clobber) the master bounds so earlier layers' contributions survive.
    *min_x = std::min(*min_x, robot_x - maximum_range_);
    *min_y = std::min(*min_y, robot_y - maximum_range_);
    *max_x = std::max(*max_x, robot_x + maximum_range_);
    *max_y = std::max(*max_y, robot_y + maximum_range_);

    auto parent = layered_costmap_->getCostmap();
    const bool geometry_changed =
      parent->getSizeInCellsX() != count_x_ ||
      parent->getSizeInCellsY() != count_y_ ||
      parent->getResolution() != resolution_;

    if (geometry_changed) {
      matchSize();  // recreate the buffer at the new size/resolution (locks internally)
    } else {
      // Rolling-window origin shift: roll the buffer to follow the parent so
      // accumulated evidence stays at its world location (grid_map::move()).
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (buffer_) {
        buffer_->move(parentCenter(parent));
      }
    }

    // Decay accumulated evidence toward the prior once per cycle.
    if (auto node = node_.lock()) {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (buffer_) {
        buffer_->decay(node->now().seconds());
      }
    }
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!buffer_) {
      return;
    }
    // Stamp LETHAL into the master where the buffer has crossed the threshold.
    // Leave everything else untouched (other layers + inflation own free/unknown).
    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        double wx, wy;
        master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
        if (buffer_->isLethal(grid_map::Position(wx, wy))) {
          master_grid.setCost(
            static_cast<unsigned int>(i), static_cast<unsigned int>(j),
            nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }

  void matchSize() override
  {
    auto parent = layered_costmap_->getCostmap();
    std::lock_guard<std::mutex> lock(costmap_mutex_);

    origin_x_ = parent->getOriginX();
    origin_y_ = parent->getOriginY();
    resolution_ = parent->getResolution();
    count_x_ = parent->getSizeInCellsX();
    count_y_ = parent->getSizeInCellsY();

    const double size_x = count_x_ * resolution_;
    const double size_y = count_y_ * resolution_;
    buffer_ = std::make_unique<sea_surface_segmentation::OccupancyBuffer>(
      size_x, size_y, resolution_, parentCenter(parent), params_);
    RCLCPP_INFO_STREAM(
      logger_, "SeaSurfaceLayer occupancy buffer sized to " << count_x_ << "x" << count_y_);
  }

private:
  // World position of the parent costmap's center (grid_map is positioned by
  // its center; nav2 Costmap2D by its lower-left origin).
  static grid_map::Position parentCenter(nav2_costmap_2d::Costmap2D * parent)
  {
    return grid_map::Position(
      parent->getOriginX() + parent->getSizeInCellsX() * parent->getResolution() / 2.0,
      parent->getOriginY() + parent->getSizeInCellsY() * parent->getResolution() / 2.0);
  }

  void segmentsCallback(const sensor_msgs::msg::Image::SharedPtr segments_msg)
  {
    // Pin the camera model under the lock, then project off-lock. cameraInfoCallback
    // swaps in a *fresh* model (never mutates in place), so this local copy is an
    // immutable snapshot — no torn read / use-after-free during projection.
    std::shared_ptr<image_geometry::PinholeCameraModel> camera_model;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      camera_model = camera_model_;
    }
    if (!camera_model) {
      return;
    }
    try {
      // Camera pose in the world (costmap global) frame: translation = optical
      // center, rotation = camera-optical → world.
      const auto tf = tf_->lookupTransform(
        global_frame_id_, segments_msg->header.frame_id, segments_msg->header.stamp,
        std::chrono::seconds(1));
      const auto image = cv_bridge::toCvShare(segments_msg, "rgb8");

      const auto & t = tf.transform.translation;
      const auto & q = tf.transform.rotation;
      const cv::Vec3d camera_origin(t.x, t.y, t.z);
      const cv::Matx33d rotation_cam_to_world =
        sea_surface_segmentation::rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w);

      // Water surface is z=0 in the tide-tracked global frame; the boat floats,
      // so this holds at any tide (see #19 / #10 H discussion).
      const auto observations = sea_surface_segmentation::project_observations(
        image->image, *camera_model, camera_origin, rotation_cam_to_world, maximum_range_, 0.0);

      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!buffer_) {
        return;
      }
      for (const auto & obs : observations) {
        const grid_map::Position p(obs.x, obs.y);
        if (obs.obstacle) {
          buffer_->hit(p);
        } else {
          buffer_->miss(p);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_STREAM(logger_, e.what());
    }
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
  {
    // Build a fresh model off-lock, then swap the pointer under the lock. Readers
    // that already copied the old pointer keep using an unchanged object — no
    // in-place mutation of a model a concurrent segmentsCallback may be reading.
    auto model = std::make_shared<image_geometry::PinholeCameraModel>();
    model->fromCameraInfo(*camera_info_msg);
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    camera_model_ = model;
  }

  std::string global_frame_id_;

  // Parent-costmap geometry tracking (to detect true size/resolution changes
  // vs rolling-origin shifts).
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double resolution_ = 1.0;
  unsigned int count_x_ = 0;
  unsigned int count_y_ = 0;

  double maximum_range_ = 100.0;
  sea_surface_segmentation::OccupancyParams params_;
  std::unique_ptr<sea_surface_segmentation::OccupancyBuffer> buffer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr segments_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;

  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model_;

  std::mutex costmap_mutex_;
};

}  // namespace sea_surface_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sea_surface_layer::SeaSurfaceLayer, nav2_costmap_2d::Layer)
