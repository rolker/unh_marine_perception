#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "depthai/depthai.hpp"
#include "depthai_marine/ffmpeg_publisher.hpp"
#include "depthai_marine/image_publisher.hpp"

namespace depthai_marine {

// Bundled per-camera configuration. Callers populate from ROS params and
// pass to `CameraBase::applyParams` before `initialize`.
struct CameraParams
{
  int preview_width = 1280;
  int preview_height = 720;
  int video_width = 1280;
  int video_height = 720;
  float fps = 5.0f;
  bool enable_video = true;

  // H.265 / H.264 on-device encoding. Off by default.
  bool h265_enable = false;
  int h265_bitrate_kbps = 4000;
  int h265_keyframe_frequency_frames = 30;
  std::string h265_profile = "H265_MAIN";
};

class CameraBase
{
public:
  CameraBase(std::shared_ptr<rclcpp::Node> node);

  virtual void initialize(std::string id, std::string label);
  void applyParams(const CameraParams & params);
  void setPreviewSize(int width, int height);
  void setVideoSize(int width, int height);
  void enableVideo(bool enable);
  void setFps(float fps);

  // H.265 (and H.264) on-device encoding knobs. Off by default — callers
  // opt in via the corresponding ROS params. See docs/h265_transport.md.
  void enableH265(bool enable);
  void setH265BitrateKbps(int bitrate_kbps);
  void setH265KeyframeFrequencyFrames(int frames);
  void setH265Profile(const std::string & profile);

  // Parses a profile name ("H265_MAIN", "H264_MAIN", "H264_BASELINE",
  // "H264_HIGH") into the DepthAI enum. Throws std::invalid_argument on
  // unknown names.
  static dai::VideoEncoderProperties::Profile parseProfile(const std::string & name);

  // Returns the FFMPEGPacket encoding string ("hevc" / "h264") for a
  // given DepthAI profile enum.
  static std::string profileEncoding(dai::VideoEncoderProperties::Profile profile);

  virtual ~CameraBase();

  virtual std::shared_ptr<dai::Pipeline> getPipeline();

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::node::Camera> camera_;
  std::shared_ptr<ImagePublisher> camera_publisher_;
  std::shared_ptr<FFMPEGPublisher> ffmpeg_publisher_;

  int preview_width_ = 1280;
  int preview_height_ = 720;
  int video_width_ = 1280;
  int video_height_ = 720;
  bool enable_video_ = true;
  float fps_ = 5.0;

  bool h265_enable_ = false;
  int h265_bitrate_kbps_ = 4000;
  int h265_keyframe_frequency_frames_ = 30;
  std::string h265_profile_ = "H265_MAIN";
};

}  // namespace depthai_marine
