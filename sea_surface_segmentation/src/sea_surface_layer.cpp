
#include <algorithm>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "segments_apply.hpp"
#include "segments_projection.hpp"

namespace sea_surface_layer
{

class SeaSurfaceLayer: public nav2_costmap_2d::Layer
{
public:
  SeaSurfaceLayer()
  {}

  ~SeaSurfaceLayer()
  {}

  void onInitialize() override
  {
    auto node = node_.lock();

    declareParameter("maximum_range", rclcpp::ParameterValue(maximum_range_));
    node->get_parameter(name_+".maximum_range", maximum_range_);


    declareParameter("segmentation_topic", rclcpp::ParameterValue("segmentation"));
    std::string segmentation_topic;
    node->get_parameter(name_+".segmentation_topic", segmentation_topic);
    declareParameter("camera_info_topic", rclcpp::ParameterValue("camera_info"));
    std::string camera_info_topic;
    node->get_parameter(name_+".camera_info_topic", camera_info_topic);

    global_frame_id_ = layered_costmap_->getGlobalFrameID();

    // Initialize sizes before wiring subscribers. Under a multi-threaded
    // executor with pre-existing publishers, a queued segmentation message
    // could otherwise dispatch segmentsCallback before matchSize() runs,
    // hitting the same count_x_-1 underflow that reset() guards against.
    matchSize();

    segments_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
      segmentation_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&SeaSurfaceLayer::segmentsCallback, this, std::placeholders::_1)
    );

    camera_info_subscriber_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&SeaSurfaceLayer::cameraInfoCallback, this, std::placeholders::_1)
    );
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    // Skip when matchSize() has not yet run: resetMapToValue would still touch
    // cell (0,0) if count_x_/count_y_ aren't both zero, but the resizeMap call
    // in matchSize() runs first under normal lifecycle and the guard makes the
    // invariant explicit.
    if (count_x_ == 0 || count_y_ == 0) {
      return;
    }
    // resetMapToValue uses exclusive (xn, yn) — matches nav2's [min, max) convention.
    segments_costmap_.resetMapToValue(0, 0, count_x_, count_y_, nav2_costmap_2d::NO_INFORMATION);
  }

  bool isClearable() override { return false; }

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double* min_x, double* min_y,
    double* max_x, double* max_y) override
  {
    // Expand the master bounds rather than overwrite — clobbering would drop
    // bounds contributions from earlier layers in the chain (chart_layer, etc.).
    *min_x = std::min(*min_x, robot_x - maximum_range_);
    *min_y = std::min(*min_y, robot_y - maximum_range_);
    *max_x = std::max(*max_x, robot_x + maximum_range_);
    *max_y = std::max(*max_y, robot_y + maximum_range_);

    auto parent = layered_costmap_->getCostmap();

    if(parent->getSizeInCellsX() != segments_costmap_.getSizeInCellsX() ||
      parent->getSizeInCellsY() != segments_costmap_.getSizeInCellsY() ||
      parent->getOriginX() != segments_costmap_.getOriginX() ||
      parent->getOriginY() != segments_costmap_.getOriginY() ||
      parent->getResolution() != segments_costmap_.getResolution()
    )
    {
      matchSize();
    }
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j, int max_i, int max_j)  override
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    apply_segments_to_master(segments_costmap_, master_grid, min_i, min_j, max_i, max_j);
  }

  void matchSize() override
  {
    RCLCPP_INFO_STREAM(logger_, "Matching size of SeaSurfaceLayer to parent costmap");
    auto parent = layered_costmap_->getCostmap();

    // Lock around resizeMap — it deletes and reallocates the costmap buffer,
    // racing with segmentsCallback's setCost loop on rolling-window costmaps
    // where updateBounds triggers matchSize on every origin shift (#6).
    std::lock_guard<std::mutex> lock(costmap_mutex_);

    origin_x_ = parent->getOriginX();
    origin_y_ = parent->getOriginY();
    resolution_ = parent->getResolution();
    count_x_ = parent->getSizeInCellsX();
    count_y_ = parent->getSizeInCellsY();

    segments_costmap_.resizeMap(count_x_, count_y_, resolution_, origin_x_, origin_y_);
  }

private:
  std::string global_frame_id_;

  double update_timeout_ = 0.5;

  void segmentsCallback(const sensor_msgs::msg::Image::SharedPtr segments_msg)
  {
    if(camera_model_)
    {

      try
      {
      
        auto transform = tf_->lookupTransform(
          segments_msg->header.frame_id, global_frame_id_, segments_msg->header.stamp, std::chrono::seconds(1));

        auto image = cv_bridge::toCvShare(segments_msg, "rgb8");


        std::lock_guard<std::mutex> lock(costmap_mutex_);

        // Defensive guard: matchSize() runs before subscribers are created,
        // but keep the invariant local so future reorderings can't reintroduce
        // a zero-size access.
        if (count_x_ == 0 || count_y_ == 0) {
          return;
        }
        // resetMapToValue uses exclusive (xn, yn) — matches nav2's [min, max) convention.
        segments_costmap_.resetMapToValue(0, 0, count_x_, count_y_, nav2_costmap_2d::NO_INFORMATION);

        for(unsigned int i = 0; i < count_x_; i++)
        {
          for(unsigned int j = 0; j < count_y_; j++)
          {
            double map_x, map_y;
            segments_costmap_.mapToWorld(i, j, map_x, map_y);
            geometry_msgs::msg::PointStamped point_in_map;
            point_in_map.point.x = map_x;
            point_in_map.point.y = map_y;
            point_in_map.point.z = 0.0;
            point_in_map.header.frame_id = global_frame_id_;
            point_in_map.header.stamp = segments_msg->header.stamp;
            geometry_msgs::msg::PointStamped point_in_camera;
            tf2::doTransform(point_in_map, point_in_camera, transform);

            if(point_in_camera.point.z < 0.0)
            {
              continue; // Skip points behind the camera
            }

            auto ray_length = sqrt(point_in_camera.point.x * point_in_camera.point.x +
                                      point_in_camera.point.y * point_in_camera.point.y +
                                      point_in_camera.point.z * point_in_camera.point.z);

            if(ray_length > maximum_range_)
            {
              continue;
            }

            auto pixel = camera_model_->project3dToPixel(cv::Point3d(
              point_in_camera.point.x, point_in_camera.point.y, point_in_camera.point.z));
            
            if(pixel.x >= 0 && pixel.x < static_cast<int>(image->image.cols) &&
               pixel.y >= 0 && pixel.y < static_cast<int>(image->image.rows))
            {
              auto pixel_value = image->image.at<cv::Vec3b>(pixel);
              // Segmentation channel convention: dominant R marks non-water
              // (lethal obstacle); dominant G/B marks water (free space).
              if(sea_surface_segmentation::is_obstacle_pixel(pixel_value))
              {
                // Only the waterline contact is a trustworthy z=0 footprint. An
                // obstacle's above-water body pixels back-project far beyond the
                // real obstacle, smearing a false radial "shadow" of lethal
                // cells out toward maximum_range. Mark only the contact lethal;
                // leave the rest at NO_INFORMATION (occluded/unknown — the value
                // reset at the start of this callback).
                if(sea_surface_segmentation::is_waterline_contact_pixel(
                     image->image, static_cast<int>(pixel.y), static_cast<int>(pixel.x)))
                {
                  segments_costmap_.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
                }
              }
              else
              {
                segments_costmap_.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
              }
            }
          }
        }
      }
      catch(const std::exception& e)
      {
        RCLCPP_WARN_STREAM(logger_, e.what());
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


  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double resolution_ = 1.0;
  unsigned int count_x_ = 0;
  unsigned int count_y_ = 0;

  nav2_costmap_2d::Costmap2D segments_costmap_;

  double maximum_range_ = 100.0;


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr segments_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;


  sensor_msgs::msg::CameraInfo camera_info_;
  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model_;

  std::mutex costmap_mutex_;

};

} // namespace sea_surface_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sea_surface_layer::SeaSurfaceLayer, nav2_costmap_2d::Layer)
