#include "depthai_marine/camera_base.hpp"
#include <thread>
#include <chrono>

namespace depthai_marine {

CameraBase::CameraBase(std::shared_ptr<rclcpp::Node> node)
: node_(node)
{
}

void CameraBase::initialize(std::string id, std::string label)
{
  auto pipeline = getPipeline();
  
  bool connected = false;
  int retries = 5;
  
  for(int i=0; i<retries; ++i) {
      try {
          // Using dai::Device constructor with pipeline, DeviceInfo, and usb2Mode=false
          device_ = std::make_shared<dai::Device>(*pipeline, dai::DeviceInfo(id), false);
          connected = true;
          break;
      } catch (const std::runtime_error& e) {
          RCLCPP_WARN(node_->get_logger(), "%s: Failed to connect to device %s: %s. Retrying in 2s... (%d/%d)", label.c_str(), id.c_str(), e.what(), i+1, retries);
          std::this_thread::sleep_for(std::chrono::seconds(2));
      }
  }
  
  if (!connected) {
      RCLCPP_ERROR(node_->get_logger(), "%s: Failed to connect after %d retries.", label.c_str(), retries);
      throw std::runtime_error("Failed to connect to device");
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), label << ": Connected to device: " <<  device_->getDeviceInfo().toString());

  if (enable_video_) {
    camera_publisher_ = std::make_shared<ImagePublisher>(node_, device_, "camera", label);
  }
}

CameraBase::~CameraBase()
{
}

std::shared_ptr<dai::Pipeline> CameraBase::getPipeline()
{
  auto pipeline = std::make_shared<dai::Pipeline>();
  camera_ = pipeline->create<dai::node::Camera>();
  camera_->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
  camera_->setPreviewSize(preview_width_, preview_height_);
  camera_->setSize(1280, 720);
  camera_->setFps(fps_);

  if (enable_video_) {
    auto camera_xlink_out = pipeline->create<dai::node::XLinkOut>();
    camera_xlink_out->setStreamName("camera");
    camera_xlink_out->input.setBlocking(false);
    camera_->preview.link(camera_xlink_out->input);
  }

  return pipeline;
}

void CameraBase::setPreviewSize(int width, int height)
{
  preview_width_ = width;
  preview_height_ = height;
}

void CameraBase::enableVideo(bool enable)
{
  enable_video_ = enable;
}

void CameraBase::setFps(float fps)
{
  fps_ = fps;
}

} // namespace depthai_marine
