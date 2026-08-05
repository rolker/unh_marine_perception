#include "depthai_marine/camera_base.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

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
  id_ = id;
  label_ = label;
  // URDF-aligned default: `<label>_optical_frame` matches the
  // `image_geometry` / REP-103 convention for camera optical frames. This
  // is a behavioral change from the historical default (which was the bare
  // `label`) — callers that need the raw `label` must now pass it
  // explicitly. See sea_surface_segmentation for the pass-through pattern.
  resolved_frame_id_ = frame_id.empty() ? (label + "_optical_frame") : frame_id;

  if (!connectDevice()) {
    RCLCPP_ERROR(node_->get_logger(), "%s: Failed to connect after retries.", label_.c_str());
    throw std::runtime_error("Failed to connect to device");
  }

  createPublishers();
}

bool CameraBase::connectDevice()
{
  auto pipeline = getPipeline();

  const int retries = 5;
  for (int i = 0; i < retries; ++i) {
    try {
      // Using dai::Device constructor with pipeline, DeviceInfo, and usb2Mode=false
      device_ = std::make_shared<dai::Device>(*pipeline, dai::DeviceInfo(id_), false);
      RCLCPP_INFO_STREAM(node_->get_logger(), label_ << ": Connected to device: " << device_->getDeviceInfo().toString());
      return true;
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(node_->get_logger(), "%s: Failed to connect to device %s: %s. Retrying in 2s... (%d/%d)", label_.c_str(), id_.c_str(), e.what(), i + 1, retries);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }
  return false;
}

void CameraBase::createPublishers()
{
  if (enable_video_) {
    camera_publisher_ = std::make_shared<ImagePublisher>(node_, device_, "camera", label_, resolved_frame_id_);
  }

  if (h265_enable_) {
    const auto profile = parseProfile(h265_profile_);
    ffmpeg_publisher_ = std::make_shared<FFMPEGPublisher>(
      node_,
      device_,
      "ffmpeg",
      label_,
      profileEncoding(profile),
      resolved_frame_id_);
  }
}

void CameraBase::restartPipeline()
{
  std::lock_guard<std::mutex> lock(restart_mutex_);

  RCLCPP_WARN(node_->get_logger(), "%s: restarting pipeline to apply h265_bitrate_kbps=%d (expect a few seconds of stream outage)",
    label_.c_str(), h265_bitrate_kbps_.load());

  // Order matters: subclass-held queues/publishers must be released while
  // the old device is still alive, then base publishers, then the device.
  onBeforeRestart();
  ffmpeg_publisher_.reset();
  camera_publisher_.reset();
  device_.reset();

  // A connect failure here must not propagate — this runs inside an executor
  // timer callback, and an escaping exception would take down the node. A
  // transiently-absent camera (USB renegotiation, power blip) recovers on the
  // retry timer instead.
  bool connected = false;
  try {
    connected = connectDevice();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "%s: unexpected error while reconnecting: %s", label_.c_str(), e.what());
  }

  if (!connected) {
    RCLCPP_ERROR(node_->get_logger(), "%s: reconnect failed; camera stays down, retrying in 10 s", label_.c_str());
    scheduleRestart(std::chrono::milliseconds(10000));
    return;
  }

  try {
    createPublishers();
    onAfterRestart();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "%s: failed to rebuild publishers after reconnect: %s; retrying in 10 s", label_.c_str(), e.what());
    scheduleRestart(std::chrono::milliseconds(10000));
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "%s: pipeline restarted with h265_bitrate_kbps=%d", label_.c_str(), h265_bitrate_kbps_.load());
}

void CameraBase::doRestart()
{
  restartPipeline();
}

void CameraBase::scheduleRestart(std::chrono::milliseconds delay)
{
  std::lock_guard<std::mutex> lock(timer_mutex_);
  if (pending_restart_timer_) {
    pending_restart_timer_->cancel();
  }
  pending_restart_timer_ = node_->create_wall_timer(delay, [this]() {
    {
      // One-shot: cancel before running so the timer never refires while a
      // restart is in progress.
      std::lock_guard<std::mutex> timer_lock(timer_mutex_);
      if (pending_restart_timer_) {
        pending_restart_timer_->cancel();
      }
    }
    doRestart();
  });
}

bool CameraBase::validateBitrateKbps(int64_t bitrate_kbps)
{
  return bitrate_kbps > 0 && bitrate_kbps <= std::numeric_limits<int>::max();
}

void CameraBase::enableDynamicBitrate()
{
  // Pre-set hook: validation only — never mutate state here, another
  // parameter in the same atomic set operation may still be rejected.
  bitrate_validate_cb_ = node_->add_on_set_parameters_callback(
    [](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() == "h265_bitrate_kbps" && !validateBitrateKbps(p.as_int())) {
          result.successful = false;
          result.reason = "h265_bitrate_kbps must be > 0";
        }
      }
      return result;
    });

  // Post-set hook: react to the accepted value.
  bitrate_apply_cb_ = node_->add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      for (const auto & p : params) {
        if (p.get_name() != "h265_bitrate_kbps") {
          continue;
        }
        const int new_kbps = static_cast<int>(p.as_int());
        const int old_kbps = h265_bitrate_kbps_.exchange(new_kbps);
        if (new_kbps == old_kbps) {
          continue;  // no-op set — don't blank the stream for nothing
        }
        if (!h265_enable_) {
          RCLCPP_INFO(node_->get_logger(), "%s: h265_bitrate_kbps=%d stored (H.265 disabled — applies if enabled later; no restart)",
            label_.c_str(), new_kbps);
          continue;
        }
        RCLCPP_WARN(node_->get_logger(), "%s: h265_bitrate_kbps %d -> %d; scheduling pipeline restart (~3-6 s stream outage)",
          label_.c_str(), old_kbps, new_kbps);
        // Short deferral coalesces rapid successive sets into one restart
        // and moves the multi-second restart off the parameter-service path.
        scheduleRestart(std::chrono::milliseconds(100));
      }
    });
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
  if (!validateBitrateKbps(bitrate_kbps)) {
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
