#pragma once

#include <memory>
#include <string>

#include "depthai/depthai.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "rclcpp/rclcpp.hpp"

namespace depthai_marine {

// Consumes encoded ImgFrames from a VideoEncoder output queue and publishes
// them as FFMPEGPacket on `<topic>/image_raw/ffmpeg`. QoS is SensorDataQoS
// to match the `ffmpeg_image_transport` subscriber convention — dropping
// stale frames is strictly better than publisher backpressure on a lossy
// LTE / udp_bridge link.
class H265Publisher
{
public:
  H265Publisher(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<dai::Device> device,
    const std::string & queue_name,
    const std::string & topic_name,
    const std::string & frame_id,
    int width,
    int height,
    const std::string & encoding);

  ~H265Publisher();

private:
  void onFrame(std::shared_ptr<dai::ImgFrame> frame);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<dai::DataOutputQueue> queue_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr publisher_;
  std::string frame_id_;
  std::string encoding_;
  int width_;
  int height_;
  int callback_id_;
};

}  // namespace depthai_marine
