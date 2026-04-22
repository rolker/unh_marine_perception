#include "depthai_marine/h265_publisher.hpp"

#include <chrono>
#include <cstring>

namespace depthai_marine {

namespace {

// FFmpeg AVPacket flag for keyframe (mirrors libavcodec's AV_PKT_FLAG_KEY
// value without pulling in libavcodec headers).
constexpr uint8_t kPktFlagKey = 0x01;

// Finds the start of the first NAL unit past the Annex-B start code
// (00 00 00 01 or 00 00 01). Returns nullptr if not found.
const uint8_t * findFirstNalHeader(const uint8_t * data, size_t size)
{
  for (size_t i = 0; i + 3 < size; ++i) {
    if (data[i] == 0x00 && data[i + 1] == 0x00) {
      if (data[i + 2] == 0x01) {
        return &data[i + 3];
      }
      if (data[i + 2] == 0x00 && data[i + 3] == 0x01 && i + 4 < size) {
        return &data[i + 4];
      }
    }
  }
  return nullptr;
}

// Inspects the first NAL unit header to determine whether the frame is a
// keyframe. H.264 IDR = NAL type 5. HEVC IRAP = NAL types 16-23 (BLA, IDR,
// CRA). Returns false on unrecognized encoding or malformed bitstream.
bool isKeyframe(const uint8_t * data, size_t size, const std::string & encoding)
{
  const uint8_t * nal = findFirstNalHeader(data, size);
  if (nal == nullptr) {
    return false;
  }
  if (encoding == "hevc") {
    const uint8_t nal_type = (nal[0] >> 1) & 0x3F;
    return nal_type >= 16 && nal_type <= 23;
  }
  if (encoding == "h264") {
    const uint8_t nal_type = nal[0] & 0x1F;
    return nal_type == 5;
  }
  return false;
}

}  // namespace

H265Publisher::H265Publisher(
  std::shared_ptr<rclcpp::Node> node,
  std::shared_ptr<dai::Device> device,
  const std::string & queue_name,
  const std::string & topic_name,
  const std::string & frame_id,
  int width,
  int height,
  const std::string & encoding)
: node_(node),
  frame_id_(frame_id),
  encoding_(encoding),
  width_(width),
  height_(height),
  callback_id_(-1)
{
  publisher_ = node_->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
    topic_name + "/image_raw/ffmpeg",
    rclcpp::SensorDataQoS());

  queue_ = device->getOutputQueue(queue_name, 5, false);
  callback_id_ = queue_->addCallback(
    [this](std::shared_ptr<dai::ADatatype> data) {
      auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
      if (frame) {
        onFrame(frame);
      }
    });
}

H265Publisher::~H265Publisher()
{
  if (queue_ && callback_id_ >= 0) {
    queue_->removeCallback(callback_id_);
  }
}

void H265Publisher::onFrame(std::shared_ptr<dai::ImgFrame> frame)
{
  const auto & data = frame->getData();
  if (data.empty()) {
    return;
  }

  ffmpeg_image_transport_msgs::msg::FFMPEGPacket msg;
  msg.header.frame_id = frame_id_;

  // Match header.stamp to getTimestamp() — same source the sibling
  // sensor_msgs/Image uses via ImageConverter::toRosMsg, so subscribers
  // can cross-reference the two time signals on the packet.
  const auto tp = frame->getTimestamp();
  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
    tp.time_since_epoch()).count();
  msg.header.stamp = rclcpp::Time(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      tp.time_since_epoch()).count(),
    RCL_STEADY_TIME);

  msg.width = width_;
  msg.height = height_;
  msg.encoding = encoding_;
  msg.pts = static_cast<uint64_t>(us);
  msg.flags = isKeyframe(data.data(), data.size(), encoding_) ? kPktFlagKey : 0;
  msg.is_bigendian = false;
  msg.data.assign(data.begin(), data.end());

  publisher_->publish(msg);
}

}  // namespace depthai_marine
