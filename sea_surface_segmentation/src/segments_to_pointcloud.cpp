#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "image_geometry/pinhole_camera_model.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "cv_bridge/cv_bridge.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "segments_projection.hpp"


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
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    // Optional override that lets a parallel instance project into a
    // failure-stage-independent frame (e.g. `bizzy/base_link_level`
    // for the nav2_collision_monitor reflex feed). When non-empty,
    // both the TF lookup and the output `header.frame_id` use this
    // frame instead of `map_frame`.
    target_frame_ = declare_parameter<std::string>("target_frame", "");

    // z-coordinate of the projection plane in whichever frame is
    // used. 0.0 matches the historical map-frame ground-plane
    // assumption and is the right starting point for base_link_level
    // mode too — the hull-floor-vs-waterline offset is treated as
    // part of the Collision Monitor polygon-sizing budget; this
    // param exists for tuning if field data demands it.
    projection_plane_z_ = declare_parameter<double>("projection_plane_z", 0.0);

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

    return LifecycleNode::on_configure(state);
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
  {
    return LifecycleNode::on_activate(state);
  }


  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
  {
    return LifecycleNode::on_deactivate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
  {
    return LifecycleNode::on_cleanup(state);
  }

private:
  void segmentsCallback(const sensor_msgs::msg::Image::SharedPtr segments_msg)
  {
    if (!camera_model_) {
      return;
    }

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

      const tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      const tf2::Matrix3x3 m(q);
      cv::Matx33d rotation;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          rotation(i, j) = m[i][j];
        }
      }

      const auto image = cv_bridge::toCvShare(segments_msg, "rgb8");
      const auto points = sea_surface_segmentation::project_obstacle_pixels(
        image->image, *camera_model_, camera_origin, rotation, projection_plane_z_);

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
    } catch (const std::exception & e) {
      RCLCPP_WARN_STREAM(get_logger(), e.what());
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
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto segments_to_pointcloud = std::make_shared<SegmentsToPointCloud>();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(segments_to_pointcloud->get_node_base_interface());
  exe.spin();


  rclcpp::shutdown();
  return 0;
}
