#pragma once

#include <memory>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "rclcpp/rclcpp.hpp"

namespace depthai_marine {

// Consumes EncodedFrames from a VideoEncoder output queue and publishes them
// as FFMPEGPacket on `<topic>/image_raw/ffmpeg`. Codec-agnostic — carries
// H.265 or H.264 depending on the profile passed to the DepthAI VideoEncoder
// (the `encoding` ctor arg labels the stream for the ffmpeg_image_transport
// decoder: "hevc" for H.265, "h264" for H.264 profiles).
//
// Conversion goes through `dai::ros::ImageConverter::toRosFFMPEGPacket`, which
// applies the same steady-clock → ROS-time base-time calibration that the
// sibling `sensor_msgs/Image` publisher uses. Both topics' `header.stamp`
// therefore land in the same ROS time domain — essential for downstream
// cross-stream sync (bag replay, message_filters, operator dashboards).
//
// QoS is `rclcpp::SensorDataQoS()` to match the `ffmpeg_image_transport`
// subscriber convention. On a lossy LTE / udp_bridge link, dropping stale
// frames is strictly better than publisher backpressure.
class FFMPEGPublisher
{
public:
  // `frame_id` stamps `header.frame_id` on every published FFMPEGPacket.
  // When empty, falls back to `topic_name` to preserve the historical
  // default. Callers that want the URDF-aligned `<label>_optical_frame`
  // default should go through `CameraBase::initialize()`, which derives it
  // for them.
  FFMPEGPublisher(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<dai::Device> device,
    const std::string & queue_name,
    const std::string & topic_name,
    const std::string & encoding,
    const std::string & frame_id = "");

  ~FFMPEGPublisher();

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<dai::ros::ImageConverter> converter_;
  std::shared_ptr<dai::DataOutputQueue> queue_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr publisher_;
  int callback_id_;
};

}  // namespace depthai_marine
