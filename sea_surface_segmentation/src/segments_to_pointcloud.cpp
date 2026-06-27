#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "image_geometry/pinhole_camera_model.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "marine_control/control_server.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "sea_surface_segmentation/segments_projection.hpp"


class SegmentsToPointCloud : public rclcpp_lifecycle::LifecycleNode
{
public:
  SegmentsToPointCloud()
  : rclcpp_lifecycle::LifecycleNode("segments_to_pointcloud")
  {
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
  {
    // Legacy parameter — projection plane is the z=0 plane of this
    // frame. Default "map" preserves pre-refactor behavior. Used as
    // the projection target when `target_frame` is empty.
    // All declares below are guarded with has_parameter() so a managed
    // configure -> cleanup -> configure cycle does not re-declare (which throws
    // ParameterAlreadyDeclaredException, since on_cleanup does not undeclare).
    // Same intent as the diagnostic_updater guard further down.
    map_frame_ = has_parameter("map_frame")
      ? get_parameter("map_frame").as_string()
      : declare_parameter<std::string>("map_frame", "map");

    // Optional override that lets a parallel instance project into a
    // failure-stage-independent frame (e.g. `bizzy/base_link_level`
    // for the nav2_collision_monitor reflex feed). When non-empty,
    // both the TF lookup and the output `header.frame_id` use this
    // frame instead of `map_frame`.
    target_frame_ = has_parameter("target_frame")
      ? get_parameter("target_frame").as_string()
      : declare_parameter<std::string>("target_frame", "");

    // z-coordinate of the projection plane in whichever frame is
    // used. 0.0 matches the historical map-frame ground-plane
    // assumption and is the right starting point for base_link_level
    // mode too — the hull-floor-vs-waterline offset is treated as
    // part of the Collision Monitor polygon-sizing budget; this
    // param exists for tuning if field data demands it.
    projection_plane_z_ = has_parameter("projection_plane_z")
      ? get_parameter("projection_plane_z").as_double()
      : declare_parameter<double>("projection_plane_z", 0.0);

    // Confidence floor for the reflex obstacle feed. Mirrors the costmap
    // SeaSurfaceLayer's obstacle_prob_min: an obstacle pixel is projected only
    // if P(obstacle) = R/(R+G+B) >= this. Default 0.0 = off (historical
    // behavior, no change for existing consumers); platforms raise it
    // (BizzyBoat: 0.60) to reject low-confidence returns such as calm-water
    // reflections. Declared with a descriptor so it is rqt_reconfigure-tunable
    // now and bindable by the marine_control remote panel
    // (unh_marine_autonomy#140 / ADR-0003).
    rcl_interfaces::msg::ParameterDescriptor obstacle_prob_min_desc;
    obstacle_prob_min_desc.description =
      "Reflex confidence floor: project an obstacle pixel only if "
      "P(obstacle)=R/(R+G+B) >= this. 0.0 disables the gate. Capped below 1.0 "
      "so a single value cannot blind the reflex feed.";
    obstacle_prob_min_desc.read_only = false;
    {
      rcl_interfaces::msg::FloatingPointRange range;
      range.from_value = 0.0;
      // Capped at 0.95 (not 1.0): on a safety feed, 1.0 would drop every
      // obstacle pixel except a pure-red one, silently blinding the reflex.
      range.to_value = 0.95;
      range.step = 0.0;  // continuous — do not snap to a grid (e.g. 0.62 stays settable)
      obstacle_prob_min_desc.floating_point_range.push_back(range);
    }
    if (!has_parameter("obstacle_prob_min")) {
      declare_parameter<double>("obstacle_prob_min", 0.0, obstacle_prob_min_desc);
    }
    obstacle_prob_min_ = get_parameter("obstacle_prob_min").as_double();

    // Validate changes to the floor from rqt_reconfigure / `ros2 param set` /
    // the marine_control panel (bound in on_activate). This on-set callback is
    // the single owner of the 0.95 cap and the NaN guard (the FloatingPointRange
    // descriptor enforces the range but not NaN). It validates *only*: the
    // cached obstacle_prob_min_ is committed in the post-set callback below, so a
    // value rejected here — or by another parameter sharing the same set() —
    // can never move the cache ahead of the declared parameter. Both callbacks
    // are guarded so a configure -> cleanup -> configure cycle registers exactly
    // one of each.
    if (!param_cb_handle_) {
      param_cb_handle_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> & params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto & p : params) {
            if (p.get_name() == "obstacle_prob_min") {
              const double v = p.as_double();
              if (!std::isfinite(v) || v < 0.0 || v > 0.95) {
                result.successful = false;
                result.reason = "obstacle_prob_min must be finite and in [0, 0.95]";
              }
            }
          }
          return result;
        });
    }
    // Commit the validated value to the cache only after the set is applied, so
    // obstacle_prob_min_ (read once per frame by segmentsCallback) always tracks
    // the declared parameter — even when the change rides in a batched set().
    if (!post_set_cb_handle_) {
      post_set_cb_handle_ = add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params) {
          for (const auto & p : params) {
            if (p.get_name() == "obstacle_prob_min") {
              obstacle_prob_min_ = p.as_double();
            }
          }
        });
    }

    tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    segments_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
      "segmentation",
      rclcpp::SensorDataQoS(),
      std::bind(&SegmentsToPointCloud::segmentsCallback, this, std::placeholders::_1)
    );

    camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "segmentation/camera_info",
      rclcpp::SensorDataQoS(),
      std::bind(&SegmentsToPointCloud::cameraInfoCallback, this, std::placeholders::_1)
    );

    // Publisher uses the node's private namespace so two parallel
    // instances (legacy map-frame + reflex base_link_level) under the
    // same parent namespace auto-isolate by node name. Downstream
    // consumers of the legacy topic name migrate in coordinated
    // follow-up PRs.
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/pointcloud",
      rclcpp::SensorDataQoS()
    );

    // Health monitoring. The updater owns its own 1 Hz timer and publishes
    // a DiagnosticArray on /diagnostics, where the operator-station
    // annunciator picks it up — so a degraded reflex feed (no camera_info,
    // TF lookups failing, every ray non-finite) is observable rather than
    // just a silently empty cloud. Created once (guarded) so a
    // configure→cleanup→configure cycle doesn't re-declare its `period`
    // parameter.
    if (!diagnostic_updater_) {
      diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
      diagnostic_updater_->setHardwareID(get_name());
      diagnostic_updater_->add(
        "obstacle projection feed",
        std::bind(&SegmentsToPointCloud::produceDiagnostics, this, std::placeholders::_1));
    }

    return LifecycleNode::on_configure(state);
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
  {
    // Expose the reflex confidence floor on the marine_control device panel so
    // the operator can retune it live, bridgeable boat->operator
    // (unh_marine_autonomy#140 / ADR-0003). Built here — not in on_configure —
    // so the control channel exists only while the reflex feed is active
    // (lifecycle gating per control_server.hpp), and torn down in on_deactivate.
    //
    // Safety (ADR-0003 D8.3): this rides the base fire-and-forget change
    // channel. That is acceptable only because obstacle_prob_min's own on-set
    // callback caps the value at 0.95, so an operator setting cannot blind the
    // reflex feed. Stronger confirm/audit is the documented marine_control
    // follow-up, not done here.
    marine_control::ControlServerOptions control_opts;
    control_opts.device_name = "Reflex Obstacle Filter";
    control_server_ = std::make_shared<marine_control::ControlServer>(this, control_opts);
    control_server_->bind_parameter("obstacle_prob_min", "P", "reflex");
    return LifecycleNode::on_activate(state);
  }


  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
  {
    // Tear down the control channel so the panel stops advertising a control
    // for an inactive reflex, and the heartbeat timer/change sub are gone.
    // Safe to destroy here: the node runs on a SingleThreadedExecutor, so this
    // callback never races an in-flight ControlServer callback (control_server.hpp
    // threading contract).
    control_server_.reset();
    return LifecycleNode::on_deactivate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
  {
    // Idempotent: deactivate already reset the server on the active->inactive
    // path; reset again so a configure->cleanup (never-activated) path also
    // leaves no dangling server.
    control_server_.reset();
    return LifecycleNode::on_cleanup(state);
  }

private:
  void segmentsCallback(const sensor_msgs::msg::Image::SharedPtr segments_msg)
  {
    if (!camera_model_) {
      // No CameraInfo yet — the projection can't run. Surface it (throttled)
      // so a never-arriving camera_info isn't an invisible dead feed; the
      // /diagnostics task reports the same state for the annunciator.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no camera_info received yet; obstacle projection idle");
      return;
    }

    ++frames_received_;
    last_frame_time_ = now();

    // Output-contract pivot: when `target_frame` is non-empty, the
    // projection lookup and the output `header.frame_id` both use it
    // instead of `map_frame`. Failure-stage independence depends on
    // *both* being the same frame — Collision Monitor will retransform
    // to its own working frame each cycle, and `transform_tolerance`
    // covers the small TF age between segments and current TF.
    const std::string & projection_frame =
      target_frame_.empty() ? map_frame_ : target_frame_;

    try {
      const auto transform = tf_buffer_->lookupTransform(
        projection_frame, segments_msg->header.frame_id, segments_msg->header.stamp,
        std::chrono::seconds(1));

      const cv::Vec3d camera_origin(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);

      // Quaternion → cam-to-target rotation. The helper normalizes the
      // quaternion and matches tf2's matrix convention (cross-checked in
      // the unit tests), so this is equivalent to the previous
      // tf2::Matrix3x3 path with added robustness against a non-unit input.
      const cv::Matx33d rotation =
        sea_surface_segmentation::rotation_matrix_from_quaternion(
          transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z,
          transform.transform.rotation.w);

      const auto image = cv_bridge::toCvShare(segments_msg, "rgb8");
      sea_surface_segmentation::ProjectionStats stats;
      const auto points = sea_surface_segmentation::project_obstacle_pixels(
        image->image, *camera_model_, camera_origin, rotation, projection_plane_z_, &stats,
        obstacle_prob_min_);
      nonfinite_dropped_ += stats.dropped_nonfinite;
      low_confidence_dropped_ += stats.dropped_low_confidence;

      pcl::PointCloud<pcl::PointXYZI> cloud;
      cloud.reserve(points.size());
      for (const auto & p : points) {
        pcl::PointXYZI point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.intensity = p.intensity;
        cloud.push_back(point);
      }

      sensor_msgs::msg::PointCloud2 pointcloud_msg;
      pcl::toROSMsg(cloud, pointcloud_msg);
      pointcloud_msg.header.frame_id = projection_frame;
      pointcloud_msg.header.stamp = segments_msg->header.stamp;
      pointcloud_publisher_->publish(pointcloud_msg);

      ++clouds_published_;
      last_publish_time_ = now();
    } catch (const std::exception & e) {
      ++tf_failures_;
      last_tf_failure_time_ = now();
      last_error_ = e.what();
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "projection failed (frame '" << projection_frame << "'): " << e.what());
    }
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
  {
    camera_info_ = *camera_info_msg;
    if(!camera_model_) {
      camera_model_ = std::make_shared<image_geometry::PinholeCameraModel>();
    }
    camera_model_->fromCameraInfo(camera_info_);
  }

  // Reports feed health on /diagnostics. Runs from the updater's 1 Hz timer
  // on the same single-threaded executor as the callbacks, so the counters
  // it reads need no synchronization. Severity is intentionally conservative
  // — WARN, never ERROR — so a transient TF gap or a not-yet-calibrated
  // camera doesn't raise a hard alarm on the annunciator; escalation to
  // ERROR (and any sustained-failure timing) is left to the operator-side
  // annunciator thresholds.
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;

    const std::string projection_frame =
      target_frame_.empty() ? map_frame_ : target_frame_;

    stat.add("mode", target_frame_.empty() ? "legacy (map_frame)" : "reflex (target_frame)");
    stat.add("projection_frame", projection_frame);
    stat.add("projection_plane_z", projection_plane_z_);
    stat.add("camera_info_received", camera_model_ ? "true" : "false");
    stat.add("segmentation_frames_received", static_cast<int>(frames_received_));
    stat.add("clouds_published", static_cast<int>(clouds_published_));
    stat.add("tf_lookup_failures", static_cast<int>(tf_failures_));
    stat.add("nonfinite_points_dropped", static_cast<int>(nonfinite_dropped_));
    stat.add("obstacle_prob_min", obstacle_prob_min_);
    stat.add("low_confidence_points_dropped", static_cast<int>(low_confidence_dropped_));

    if (frames_received_ > 0) {
      stat.add("seconds_since_last_frame", (now() - last_frame_time_).seconds());
    }
    if (clouds_published_ > 0) {
      stat.add("seconds_since_last_publish", (now() - last_publish_time_).seconds());
    }
    if (tf_failures_ > 0) {
      stat.add("last_error", last_error_);
    }

    if (!camera_model_) {
      stat.summary(DiagnosticStatus::WARN, "waiting for camera_info");
      return;
    }
    if (frames_received_ == 0) {
      stat.summary(DiagnosticStatus::WARN, "no segmentation frames received yet");
      return;
    }
    // Frames are arriving but the latest ones aren't producing a cloud: the
    // TF lookup for the projection frame is failing, so the obstacle stream
    // is currently dead. The short-circuit guards the cross-clock time
    // comparison — last_*_time_ are only compared once both counters are
    // non-zero, by which point both were stamped from the same node clock.
    const bool projection_failing =
      clouds_published_ == 0 ||
      (tf_failures_ > 0 && last_tf_failure_time_ > last_publish_time_);
    if (projection_failing) {
      stat.summary(
        DiagnosticStatus::WARN,
        "segmentation arriving but projection failing; check TF for '" +
        projection_frame + "'");
      return;
    }
    stat.summary(DiagnosticStatus::OK, "projecting obstacles");
  }


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr segments_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

  sensor_msgs::msg::CameraInfo camera_info_;
  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string map_frame_;
  std::string target_frame_;
  double projection_plane_z_{0.0};
  double obstacle_prob_min_{0.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_cb_handle_;

  // Boat-side device-control server: publishes obstacle_prob_min as a bridgeable
  // marine_control, applies operator changes via the bound parameter (whose
  // on-set validation still owns the 0.95 cap). Lives only while active; see
  // on_activate/on_deactivate.
  std::shared_ptr<marine_control::ControlServer> control_server_;

  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;

  // Health counters for /diagnostics. Read/written only from the
  // single-threaded executor (callbacks + updater timer), so no
  // synchronization is needed.
  std::size_t frames_received_{0};
  std::size_t clouds_published_{0};
  std::size_t tf_failures_{0};
  std::size_t nonfinite_dropped_{0};
  std::size_t low_confidence_dropped_{0};
  rclcpp::Time last_frame_time_;
  rclcpp::Time last_publish_time_;
  rclcpp::Time last_tf_failure_time_;
  std::string last_error_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto segments_to_pointcloud = std::make_shared<SegmentsToPointCloud>();

  // The SingleThreadedExecutor is load-bearing for the marine_control
  // ControlServer threading contract (control_server.hpp): bind_parameter() runs
  // in on_activate and reset() in on_deactivate, both on this executor thread
  // while spinning. A single thread cannot dispatch the server's heartbeat/change
  // callbacks concurrently with those transitions, so the unlocked binding table
  // is safe. Switching to a MultiThreadedExecutor (or composing this node into a
  // multi-threaded container) would void that guarantee — don't, without revisiting.
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(segments_to_pointcloud->get_node_base_interface());
  exe.spin();


  rclcpp::shutdown();
  return 0;
}
