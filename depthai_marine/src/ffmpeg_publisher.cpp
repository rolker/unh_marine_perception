#include "depthai_marine/ffmpeg_publisher.hpp"

namespace depthai_marine {

FFMPEGPublisher::FFMPEGPublisher(
  std::shared_ptr<rclcpp::Node> node,
  std::shared_ptr<dai::Device> device,
  const std::string & queue_name,
  const std::string & topic_name,
  const std::string & encoding)
: node_(node),
  callback_id_(-1)
{
  // Construct an ImageConverter with the same `interleaved=true` setting the
  // sibling ImagePublisher uses. The converter captures its ROS-time /
  // steady-clock base offset at construction; as long as both clocks advance
  // at the same rate (they do on Linux), two converters constructed moments
  // apart produce matching stamps for any given device frame.
  converter_ = std::make_shared<dai::ros::ImageConverter>(topic_name, true);
  converter_->setFFMPEGEncoding(encoding);

  publisher_ = node_->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
    topic_name + "/image_raw/ffmpeg",
    rclcpp::SensorDataQoS());

  queue_ = device->getOutputQueue(queue_name, 5, false);
  callback_id_ = queue_->addCallback(
    [this](std::shared_ptr<dai::ADatatype> data) {
      auto frame = std::dynamic_pointer_cast<dai::EncodedFrame>(data);
      if (frame) {
        // Move-based publish: avoids copying the encoded packet (can be
        // hundreds of kB for video) before DDS serialization.
        auto packet = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
          converter_->toRosFFMPEGPacket(frame));
        publisher_->publish(std::move(packet));
      }
    });
}

FFMPEGPublisher::~FFMPEGPublisher()
{
  if (queue_ && callback_id_ >= 0) {
    queue_->removeCallback(callback_id_);
  }
}

}  // namespace depthai_marine
