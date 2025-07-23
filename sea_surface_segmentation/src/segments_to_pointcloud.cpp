#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "cv_bridge/cv_bridge.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


class SegmentsToPointCloud : public rclcpp_lifecycle::LifecycleNode
{
public:
  SegmentsToPointCloud()
  : rclcpp_lifecycle::LifecycleNode("segments_to_pointcloud")
  {
    // Constructor implementation
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
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

    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "segmentation/pointcloud",
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
    if(camera_model_)
    {

      try
      {
      
        auto transform = tf_buffer_->lookupTransform(
          map_frame_, segments_msg->header.frame_id, segments_msg->header.stamp, std::chrono::seconds(1));

        geometry_msgs::msg::PoseStamped camera_origin;
        camera_origin.header = segments_msg->header;
        camera_origin.pose.orientation.w = 1.0; // Identity orientation
        geometry_msgs::msg::PoseStamped camera_origin_map;
        tf2::doTransform(camera_origin, camera_origin_map, transform);
        auto p1 = camera_origin_map.pose.position;


        auto image = cv_bridge::toCvShare(segments_msg, "rgb8");

        std::vector<cv::Point2d> target_pixels;

        pcl::PointCloud<pcl::PointXYZI>::Ptr targets(
          new pcl::PointCloud<pcl::PointXYZI>);
        targets->header.frame_id = map_frame_;

        for(int row = 0; row < image->image.rows; ++row)
        {
          for(int col = 0; col < image->image.cols; ++col)
          {
            auto pixel = cv::Point2d(col, row);
            auto pixel_value = image->image.at<cv::Vec3b>(row, col);
            if(pixel_value[0] > pixel_value[1] && pixel_value[0] > pixel_value[2])
            {
              target_pixels.push_back(pixel);
            }
          }
        }
    
        for(const auto & pixel: target_pixels)
        {
          //RCLCPP_INFO_STREAM(get_logger(), "Processing pixel: " << pixel.x << ", " << pixel.y);
          auto ray = camera_model_->projectPixelTo3dRay(pixel);
          //RCLCPP_INFO_STREAM(get_logger(), "Ray: " << ray.x << ", " << ray.y << ", " << ray.z);
          geometry_msgs::msg::PoseStamped ray_pose;
          ray_pose.header = segments_msg->header;
          ray_pose.pose.position.x = ray.x;
          ray_pose.pose.position.y = ray.y;
          ray_pose.pose.position.z = ray.z;
          ray_pose.pose.orientation.w = 1.0;
    
          geometry_msgs::msg::PoseStamped ray_pose_map;
          tf2::doTransform(ray_pose, ray_pose_map, transform);
          auto p2 = ray_pose_map.pose.position;
          //RCLCPP_INFO_STREAM(get_logger(), "Ray end in map frame: "
          //    << p2.x << ", " << p2.y << ", " << p2.z);

          // ground plane eq: z=0
          // line eq: P=p1+u(p2-p1)
          // P.z = p1.z+u(p2.z-p1.z) = 0
          // u = -p1.z/(p2.z-p1.z)

          double u = -p1.z / (p2.z - p1.z);
          if(u>0.0)
          {
            auto px = p1.x+ u * (p2.x - p1.x);
            auto py = p1.y+ u * (p2.y - p1.y);

            pcl::PointXYZI point;
            point.x = px;
            point.y = py;
            point.z = 0.0;
            point.intensity = 1.0;
            //RCLCPP_INFO_STREAM(get_logger(), "Intersecting point in map frame: "
            //    << point.x << ", " << point.y << ", " << point.z);
            targets->push_back(point);
          }
        }

        pcl_conversions::toPCL(segments_msg->header.stamp, targets->header.stamp);


        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        pcl::toROSMsg(*targets, pointcloud_msg);
        pointcloud_msg.header.frame_id = map_frame_;
        pointcloud_msg.header.stamp = segments_msg->header.stamp;
        pointcloud_publisher_->publish(pointcloud_msg);
      }
      catch(const std::exception& e)
      {
        RCLCPP_WARN_STREAM(get_logger(), e.what());
      }
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
