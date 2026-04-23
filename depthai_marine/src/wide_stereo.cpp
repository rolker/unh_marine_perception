#include "rclcpp/rclcpp.hpp"
#include "depthai_marine/camera_base.hpp"

class MainCamera: public depthai_marine::CameraBase
{

public:
  MainCamera(std::shared_ptr<rclcpp::Node> node, std::string id, const depthai_marine::CameraParams & params):
    depthai_marine::CameraBase(node)
  {
    applyParams(params);
    initialize(id, "right");
  }

  virtual std::shared_ptr<dai::Pipeline> getPipeline() override
  {
    auto pipeline = depthai_marine::CameraBase::getPipeline();
    return pipeline;
  }

  std::shared_ptr<dai::DataInputQueue> getLeftImageInQueue()
  {
    return left_image_in_queue_;
  }

private:
  std::shared_ptr<dai::DataInputQueue> left_image_in_queue_;

};

class SecondaryCamera: public depthai_marine::CameraBase
{
public:
  SecondaryCamera(std::shared_ptr<rclcpp::Node> node, std::string id, const depthai_marine::CameraParams & params):
    depthai_marine::CameraBase(node)
  {
    applyParams(params);
    initialize(id, "left");
    left_image_out_queue_ = device_->getOutputQueue("left_out", 8, false);
  }

  virtual std::shared_ptr<dai::Pipeline> getPipeline() override
  {
    auto pipeline = depthai_marine::CameraBase::getPipeline();

    auto left_xlink_out = pipeline->create<dai::node::XLinkOut>();
    left_xlink_out->setStreamName("left_out");
    left_xlink_out->input.setBlocking(false);
    camera_->preview.link(left_xlink_out->input);

    return pipeline;
  }

  std::shared_ptr<dai::DataOutputQueue> getLeftImageOutQueue()
  {
    return left_image_out_queue_;
  }

private:
  std::shared_ptr<dai::DataOutputQueue> left_image_out_queue_;

};

class WideStereo
{
public:
  WideStereo(std::shared_ptr<rclcpp::Node> node)
  : logger_(node->get_logger())
  {
    ros_base_time_ = node->get_clock()->now();
    steady_base_time_ = std::chrono::steady_clock::now();

    depthai_marine::CameraParams params;

    if(!node->has_parameter("enable_video"))
      node->declare_parameter("enable_video", params.enable_video);
    params.enable_video = node->get_parameter("enable_video").as_bool();

    if(!node->has_parameter("preview_width"))
      node->declare_parameter("preview_width", params.preview_width);
    params.preview_width = node->get_parameter("preview_width").as_int();

    if(!node->has_parameter("preview_height"))
      node->declare_parameter("preview_height", params.preview_height);
    params.preview_height = node->get_parameter("preview_height").as_int();

    if(!node->has_parameter("video_width"))
      node->declare_parameter("video_width", params.video_width);
    params.video_width = node->get_parameter("video_width").as_int();

    if(!node->has_parameter("video_height"))
      node->declare_parameter("video_height", params.video_height);
    params.video_height = node->get_parameter("video_height").as_int();

    if(!node->has_parameter("fps"))
      node->declare_parameter("fps", static_cast<double>(params.fps));
    params.fps = static_cast<float>(node->get_parameter("fps").as_double());

    if(!node->has_parameter("h265_enable"))
      node->declare_parameter("h265_enable", params.h265_enable);
    params.h265_enable = node->get_parameter("h265_enable").as_bool();

    if(!node->has_parameter("h265_bitrate_kbps"))
      node->declare_parameter("h265_bitrate_kbps", params.h265_bitrate_kbps);
    params.h265_bitrate_kbps = node->get_parameter("h265_bitrate_kbps").as_int();

    if(!node->has_parameter("h265_keyframe_frequency_frames"))
      node->declare_parameter("h265_keyframe_frequency_frames", params.h265_keyframe_frequency_frames);
    params.h265_keyframe_frequency_frames = node->get_parameter("h265_keyframe_frequency_frames").as_int();

    if(!node->has_parameter("h265_profile"))
      node->declare_parameter("h265_profile", params.h265_profile);
    params.h265_profile = node->get_parameter("h265_profile").as_string();

    if(!node->has_parameter("right_camera_id"))
      node->declare_parameter("right_camera_id", std::string(""));
    auto right_camera_id = node->get_parameter("right_camera_id").as_string();
    right_camera_ = std::make_shared<MainCamera>(node, right_camera_id, params);

    if(!node->has_parameter("left_camera_id"))
      node->declare_parameter("left_camera_id", std::string(""));
    auto left_camera_id = node->get_parameter("left_camera_id").as_string();
    left_camera_ = std::make_shared<SecondaryCamera>(node, left_camera_id, params);
  }

  void forwardLeftToRight(std::shared_ptr<dai::ADatatype> left_frame_data)
  {
    auto left_frame = std::dynamic_pointer_cast<dai::ImgFrame>(left_frame_data);
    if(left_frame)
      right_camera_->getLeftImageInQueue()->send(left_frame);
  }

private:
  rclcpp::Logger logger_;
  rclcpp::Time ros_base_time_;
  std::chrono::time_point<std::chrono::steady_clock> steady_base_time_;
  int64_t total_ns_change_{0};

  std::shared_ptr<MainCamera> right_camera_;
  std::shared_ptr<SecondaryCamera> left_camera_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exe;
  auto node = std::make_shared<rclcpp::Node>("measure_timing");
  auto ws = std::make_shared<WideStereo>(node);
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
