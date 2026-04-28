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
  throw std::invalid_argument(
    "Unknown H.265/H.264 profile: " + name +
    ". Supported values: H265_MAIN, H264_MAIN, H264_BASELINE, H264_HIGH");
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

void CameraBase::initialize(std::string id, std::string label, std::string frame_id)
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

  // URDF-aligned default: `<label>_optical_frame` matches the
  // `image_geometry` / REP-103 convention for camera optical frames. This
  // is a behavioral change from the historical default (which was the bare
  // `label`) — callers that need the raw `label` must now pass it
  // explicitly. See sea_surface_segmentation for the pass-through pattern.
  const std::string resolved_frame_id = frame_id.empty() ? (label + "_optical_frame") : frame_id;

  if (enable_video_) {
    camera_publisher_ = std::make_shared<ImagePublisher>(node_, device_, "camera", label, resolved_frame_id);
  }

  if (h265_enable_) {
    const auto profile = parseProfile(h265_profile_);
    ffmpeg_publisher_ = std::make_shared<FFMPEGPublisher>(
      node_,
      device_,
      "ffmpeg",
      label,
      profileEncoding(profile),
      resolved_frame_id);
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

    auto ffmpeg_xlink_out = pipeline->create<dai::node::XLinkOut>();
    // Codec-agnostic stream name — the encoder may emit H.265 or H.264
    // depending on the configured profile.
    ffmpeg_xlink_out->setStreamName("ffmpeg");
    ffmpeg_xlink_out->input.setBlocking(false);
    // Use VideoEncoder::out (EncodedFrame) rather than ::bitstream (ImgFrame)
    // so dai::ros::ImageConverter::toRosFFMPEGPacket can do the conversion
    // — same time-base calibration as the sibling sensor_msgs/Image path.
    encoder->out.link(ffmpeg_xlink_out->input);
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
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("Video size must have positive width and height");
  }
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
  if (bitrate_kbps <= 0) {
    throw std::invalid_argument("H.265 bitrate must be > 0 kbps");
  }
  h265_bitrate_kbps_ = bitrate_kbps;
}

void CameraBase::setH265KeyframeFrequencyFrames(int frames)
{
  if (frames <= 0) {
    throw std::invalid_argument("H.265 keyframe frequency must be > 0 frames");
  }
  h265_keyframe_frequency_frames_ = frames;
}

void CameraBase::setH265Profile(const std::string & profile)
{
  h265_profile_ = profile;
}

}  // namespace depthai_marine
