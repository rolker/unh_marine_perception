#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
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

  // `frame_id` stamps `header.frame_id` on every Image / CameraInfo /
  // FFMPEGPacket emitted by the publishers this method constructs. When
  // empty, defaults to `<label>_optical_frame` (the URDF convention for
  // `image_geometry`-style optical frames). Callers that want a different
  // frame name (e.g. namespaced `bizzy/oak_forward_optical`) pass it
  // explicitly. The previous behavior — `frame_id == label` — is not
  // preserved on this path; callers that need the raw `label` must pass
  // it through explicitly.
  virtual void initialize(std::string id, std::string label, std::string frame_id = "");
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

  // Opt-in dynamic `h265_bitrate_kbps` (docs/h265_transport.md § Dynamic
  // bitrate). Registers validation + apply callbacks for the node's
  // `h265_bitrate_kbps` parameter; an accepted change schedules a deferred
  // pipeline restart (the RVC2 VideoEncoder has no runtime bitrate control).
  // Registration itself needs only the node — no device — so it is testable
  // device-free. Callers whose pipelines cannot survive a device rebuild
  // (e.g. wide_stereo's cross-device left→right forwarding queue) must NOT
  // call this; they keep the read-once-at-startup behavior.
  // Call after `initialize()` (it logs with the stored label) and before the
  // node starts spinning.
  void enableDynamicBitrate();

  // Shared validation for `h265_bitrate_kbps` — used by both
  // `setH265BitrateKbps` and the dynamic-parameter callback. int64_t so the
  // raw rclcpp parameter value can be checked before narrowing.
  static bool validateBitrateKbps(int64_t bitrate_kbps);

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
  // Restart extension points for subclasses that hold device resources
  // CameraBase does not know about (e.g. SegmentorCamera's NN output queue
  // and BridgePublisher). `onBeforeRestart()` runs while the old `device_`
  // is still alive and must release every subclass-held queue/publisher on
  // it; `onAfterRestart()` runs after the new `device_` is connected and the
  // base publishers are rebuilt, and re-acquires those resources.
  virtual void onBeforeRestart() {}
  virtual void onAfterRestart() {}

  // Deferred-restart entry point the coalescing timer calls. Virtual purely
  // as a device-free test seam — tests override it with a counter; the
  // default forwards to `restartPipeline()`.
  virtual void doRestart();

  // Tear down publishers + device and rebuild the pipeline with the current
  // member values (serialized by `restart_mutex_`). Never throws: on connect
  // failure it logs an error and re-arms the restart timer so a
  // transiently-absent camera recovers when it returns.
  void restartPipeline();

  // Build the pipeline and connect to `id_` with the 5x2s retry loop.
  // Returns false (no throw) if all retries fail.
  bool connectDevice();

  // Construct the Image / FFMPEG publishers on the current `device_` from
  // the stored identity, honoring `enable_video_` / `h265_enable_`.
  void createPublishers();

  // (Re)arm the one-shot restart timer to call `doRestart()` after `delay`.
  // Thread-safe against concurrent scheduling and the timer's own firing.
  void scheduleRestart(std::chrono::milliseconds delay);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::node::Camera> camera_;
  std::shared_ptr<ImagePublisher> camera_publisher_;
  std::shared_ptr<FFMPEGPublisher> ffmpeg_publisher_;

  // Connection identity stored by `initialize()` so `restartPipeline()` can
  // reconnect without re-parameterizing. `resolved_frame_id_` holds the
  // post-default-resolution frame id (named distinctly from
  // SegmentorCamera's own `frame_id_` to avoid shadowing).
  std::string id_;
  std::string label_;
  std::string resolved_frame_id_;

  int preview_width_ = 1280;
  int preview_height_ = 720;
  int video_width_ = 1280;
  int video_height_ = 720;
  bool enable_video_ = true;
  float fps_ = 5.0;

  bool h265_enable_ = false;
  // Atomic: written by the parameter apply-callback on one executor thread
  // while `restartPipeline()` reads it on another.
  std::atomic<int> h265_bitrate_kbps_{4000};
  int h265_keyframe_frequency_frames_ = 30;
  std::string h265_profile_ = "H265_MAIN";

private:
  // Serializes restarts (rapid `param set` bursts, retry-after-failure).
  std::mutex restart_mutex_;
  // Bitrate baked into the currently-running pipeline. Seeded by
  // `initialize()` and updated on every successful `restartPipeline()`; guarded
  // by `restart_mutex_`. Lets a restart short-circuit when the running pipeline
  // already carries the target — e.g. a set absorbed early by an in-progress
  // restart's `getPipeline()` still scheduled its own timer, which would
  // otherwise blank the stream a second time for no change.
  int applied_bitrate_kbps_{4000};
  // Guards `pending_restart_timer_` itself (scheduled from parameter
  // callbacks, replaced from the failure path, fired by the executor).
  std::mutex timer_mutex_;
  rclcpp::TimerBase::SharedPtr pending_restart_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr bitrate_validate_cb_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr bitrate_apply_cb_;
};

}  // namespace depthai_marine
