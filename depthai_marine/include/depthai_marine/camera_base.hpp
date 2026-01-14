#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "depthai/depthai.hpp"
#include "depthai_marine/image_publisher.hpp"

namespace depthai_marine {

class CameraBase
{
public:
  CameraBase(std::shared_ptr<rclcpp::Node> node);

  virtual void initialize(std::string id, std::string label);
  void setPreviewSize(int width, int height);
  void enableVideo(bool enable);
  void setFps(float fps);

  virtual ~CameraBase();

  virtual std::shared_ptr<dai::Pipeline> getPipeline();

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::node::Camera> camera_;
  std::shared_ptr<ImagePublisher> camera_publisher_;

  int preview_width_ = 1280;
  int preview_height_ = 720;
  bool enable_video_ = true;
  float fps_ = 5.0;

};

} // namespace depthai_marine
