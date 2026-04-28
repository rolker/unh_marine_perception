#pragma once

#include "rclcpp/rclcpp.hpp"
#include "depthai/depthai.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"

namespace depthai_marine {

class ImagePublisher
{
public:
  // `frame_id` stamps `header.frame_id` on every published Image / CameraInfo.
  // When empty, falls back to `topic_name` to preserve the historical default.
  // Callers that want the URDF-aligned `<label>_optical_frame` default should
  // go through `CameraBase::initialize()`, which derives it for them.
  ImagePublisher(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<dai::Device> device,
    std::string queue_name,
    std::string topic_name,
    std::string frame_id = "");

private:
  std::shared_ptr<dai::DataOutputQueue> camera_queue_;
  std::shared_ptr<dai::ros::ImageConverter> image_converter_;
  std::shared_ptr<dai::ros::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> > image_publisher_;
};

} // namespace depthai_marine
