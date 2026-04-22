#include "depthai_marine/camera_base.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

namespace depthai_marine {

CameraBase::CameraBase(std::shared_ptr<rclcpp::Node> node)
: node_(node)
{
}

dai::VideoEncoderProperties::Profile CameraBase::parseProfile(const std::string & name)
{
  if (name == "H265_MAIN") return dai::VideoEncoderProperties::Profile::H265_MAIN;
  if (name == "H264_MAIN") return dai::VideoEncoderProperties::Profile::H264_MAIN;
  if (name == "H264_BASELINE") return dai::VideoEncoderProperties::Profile::H264_BASELINE;
  if (name == "H264_HIGH") return dai::VideoEncoderProperties::Profile::H264_HIGH;
  throw std::invalid_argument("Unknown H.265/H.264 profile: " + name);
}

std::string CameraBase::profileEncoding(dai::VideoEncoderProperties::Profile profile)
{
  switch (profile) {
    case dai::VideoEncoderProperties::Profile::H265_MAIN:
      return "hevc";
    case dai::VideoEncoderProperties::Profile::H264_MAIN:
    case dai::VideoEncoderProperties::Profile::H264_BASELINE:
    case dai::VideoEncoderProperties::Profile::H264_HIGH:
      return "h264";
    default:
      throw std::invalid_argument("Profile has no FFMPEGPacket encoding mapping");
  }
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

  if (h265_enable_) {
    const auto profile = parseProfile(h265_profile_);
    h265_publisher_ = std::make_shared<H265Publisher>(
      node_,
      device_,
      "h265",
      label,
      label + "_optical_frame",
      video_width_,
      video_height_,
      profileEncoding(profile));
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
  camera_->setSize(video_width_, video_height_);
  camera_->setFps(fps_);

  if (enable_video_) {
    auto camera_xlink_out = pipeline->create<dai::node::XLinkOut>();
    camera_xlink_out->setStreamName("camera");
    camera_xlink_out->input.setBlocking(false);
    camera_->preview.link(camera_xlink_out->input);
  }

  if (h265_enable_) {
    const auto profile = parseProfile(h265_profile_);
    auto encoder = pipeline->create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(fps_, profile);
    encoder->setBitrateKbps(h265_bitrate_kbps_);
    encoder->setKeyframeFrequency(h265_keyframe_frequency_frames_);
    camera_->video.link(encoder->input);

    auto h265_xlink_out = pipeline->create<dai::node::XLinkOut>();
    h265_xlink_out->setStreamName("h265");
    h265_xlink_out->input.setBlocking(false);
    encoder->bitstream.link(h265_xlink_out->input);
  }

  return pipeline;
}

void CameraBase::applyParams(const CameraParams & params)
{
  setPreviewSize(params.preview_width, params.preview_height);
  setVideoSize(params.video_width, params.video_height);
  setFps(params.fps);
  enableVideo(params.enable_video);
  enableH265(params.h265_enable);
  setH265BitrateKbps(params.h265_bitrate_kbps);
  setH265KeyframeFrequencyFrames(params.h265_keyframe_frequency_frames);
  setH265Profile(params.h265_profile);
}

void CameraBase::setPreviewSize(int width, int height)
{
  preview_width_ = width;
  preview_height_ = height;
}

void CameraBase::setVideoSize(int width, int height)
{
  video_width_ = width;
  video_height_ = height;
}

void CameraBase::enableVideo(bool enable)
{
  enable_video_ = enable;
}

void CameraBase::setFps(float fps)
{
  fps_ = fps;
}

void CameraBase::enableH265(bool enable)
{
  h265_enable_ = enable;
}

void CameraBase::setH265BitrateKbps(int bitrate_kbps)
{
  h265_bitrate_kbps_ = bitrate_kbps;
}

void CameraBase::setH265KeyframeFrequencyFrames(int frames)
{
  h265_keyframe_frequency_frames_ = frames;
}

void CameraBase::setH265Profile(const std::string & profile)
{
  h265_profile_ = profile;
}

}  // namespace depthai_marine
