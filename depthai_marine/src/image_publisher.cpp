#include "depthai_marine/image_publisher.hpp"

namespace depthai_marine {

ImagePublisher::ImagePublisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Device> device, std::string queue_name, std::string topic_name, std::string frame_id)
{
  camera_queue_ = device->getOutputQueue(queue_name, 5, false);

  auto calibration_handler = device->readCalibration();
  // Empty frame_id → historical behavior: stamp with `topic_name`. Callers
  // that want the URDF-aligned `<label>_optical_frame` default go through
  // CameraBase::initialize(), which derives it before getting here.
  const std::string & resolved_frame_id = frame_id.empty() ? topic_name : frame_id;
  image_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(resolved_frame_id, true);

  auto camera_info = image_converter_->calibrationToCameraInfo(calibration_handler, dai::CameraBoardSocket::CAM_A, 1280, 720);

  image_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> >(
    camera_queue_,
    node,
    topic_name+"/image_raw",
    std::bind(&dai::ros::ImageConverter::toRosMsg, image_converter_.get(), std::placeholders::_1, std::placeholders::_2),
    10,
    camera_info,
    topic_name,
    false
  );

  image_publisher_->addPublisherCallback();
}

} // namespace depthai_marine
