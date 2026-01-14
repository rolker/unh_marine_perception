#include "rclcpp/rclcpp.hpp"
#include "depthai_marine/camera_base.hpp"

class MainCamera: public depthai_marine::CameraBase
{

public:
  MainCamera(std::shared_ptr<rclcpp::Node> node, std::string id):
    depthai_marine::CameraBase(node)
  {
    initialize(id, "right");
    // depth_publisher_ = std::make_shared<ImagePublisher>(node, device_, "depth", "depth");
    //left_image_in_queue_ = device_->getInputQueue("left_in");

  }

  virtual std::shared_ptr<dai::Pipeline> getPipeline() override
  {
    auto pipeline = depthai_marine::CameraBase::getPipeline();

    // auto sync = pipeline->create<dai::node::Sync>();
    // sync->setSyncThreshold(std::chrono::milliseconds(100));
    // sync->setSyncAttempts(0);
    // camera_->preview.link(sync->inputs["right"]);

    // auto left_xlink_in = pipeline->create<dai::node::XLinkIn>();
    // left_xlink_in->setStreamName("left_in");
    // left_xlink_in->out.link(sync->inputs["left"]);

    // auto images_demux = pipeline->create<dai::node::MessageDemux>();
    // sync->out.link(images_demux->input);

    // auto stereo_depth = pipeline->create<dai::node::StereoDepth>();

    // images_demux->outputs["left"].link(stereo_depth->left);
    // images_demux->outputs["right"].link(stereo_depth->right);

    // auto depth_xlink_out = pipeline->create<dai::node::XLinkOut>();
    // depth_xlink_out->setStreamName("depth");
    // depth_xlink_out->input.setBlocking(false);
    // stereo_depth->depth.link(depth_xlink_out->input);

    return pipeline;
  }

  std::shared_ptr<dai::DataInputQueue> getLeftImageInQueue()
  {
    return left_image_in_queue_;
  }

private:
  // std::shared_ptr<depthai_marine::ImagePublisher> depth_publisher_;
  std::shared_ptr<dai::DataInputQueue> left_image_in_queue_;

};

class SecondaryCamera: public depthai_marine::CameraBase
{
public:
  SecondaryCamera(std::shared_ptr<rclcpp::Node> node, std::string id):
    depthai_marine::CameraBase(node)
  {
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

    if(!node->has_parameter("right_camera_id"))
      node->declare_parameter("right_camera_id", std::string(""));
    auto right_camera_id = node->get_parameter("right_camera_id").as_string();
    right_camera_ = std::make_shared<MainCamera>(node, right_camera_id);

    if(!node->has_parameter("left_camera_id"))
      node->declare_parameter("left_camera_id", std::string(""));
    auto left_camera_id = node->get_parameter("left_camera_id").as_string();
    left_camera_ = std::make_shared<SecondaryCamera>(node, left_camera_id);

    // left_camera_->getLeftImageOutQueue()->addCallback(
    //   std::bind(&WideStereo::forwardLeftToRight, this, std::placeholders::_1)
    // );
 
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
