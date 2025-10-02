#include "rclcpp/rclcpp.hpp"
#include "depthai/depthai.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/DisparityConverter.hpp"


class ImagePublisher
{
public:
  ImagePublisher(std::shared_ptr<rclcpp::Node> node, std:: shared_ptr<dai::Device> device, std::string queue_name, std::string topic_name)
  {
    camera_queue_ = device->getOutputQueue(queue_name, 5, false);

    auto calibration_handler = device->readCalibration();
    image_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(topic_name, true);

    auto camera_info = image_converter_->calibrationToCameraInfo(calibration_handler, dai::CameraBoardSocket::CAM_A, 1280, 720);

    image_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> >(
      camera_queue_,
      node,
      topic_name+"/image_raw",
      std::bind(&dai::ros::ImageConverter::toRosMsg, image_converter_.get(), std::placeholders::_1, std::placeholders::_2),
      10,
      camera_info,
      topic_name,
      false
    );

    image_publisher_->addPublisherCallback();
  }

private:
  std::shared_ptr<dai::DataOutputQueue> camera_queue_;

  std::shared_ptr<dai::ros::ImageConverter> image_converter_;
  std::shared_ptr<dai::ros::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> > image_publisher_;
};


class CameraBase
{
public:
  CameraBase(std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  {
  }

  void initialize(std::string id, std::string label)
  {
    auto pipeline = getPipeline();
    device_ = std::make_shared<dai::Device>(*pipeline, dai::DeviceInfo(id), false);

    RCLCPP_INFO_STREAM(node_->get_logger(), label << ": Connected to device: " <<  device_->getDeviceInfo().toString());

    camera_publisher_ = std::make_shared<ImagePublisher>(node_, device_, "camera", label);
  }

  virtual ~CameraBase()
  {
  }

  virtual std::shared_ptr<dai::Pipeline> getPipeline()
  {
    auto pipeline = std::make_shared<dai::Pipeline>();
    camera_ = pipeline->create<dai::node::Camera>();
    camera_->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
    camera_->setPreviewSize(1280, 720);
    camera_->setSize(1280, 720);

    auto camera_xlink_out = pipeline->create<dai::node::XLinkOut>();
    camera_xlink_out->setStreamName("camera");
    camera_xlink_out->input.setBlocking(false);

    camera_->preview.link(camera_xlink_out->input);

    return pipeline;
  }

protected:
  std::shared_ptr<dai::node::Camera> camera_;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<dai::Device> device_;

private:
  std::shared_ptr<ImagePublisher> camera_publisher_;

};



class MainCamera: public CameraBase
{

public:
  MainCamera(std::shared_ptr<rclcpp::Node> node, std::string id):
    CameraBase(node)
  {
    initialize(id, "right");
    // depth_publisher_ = std::make_shared<ImagePublisher>(node, device_, "depth", "depth");
    //left_image_in_queue_ = device_->getInputQueue("left_in");

  }

  virtual std::shared_ptr<dai::Pipeline> getPipeline() override
  {
    auto pipeline = CameraBase::getPipeline();

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
  std::shared_ptr<ImagePublisher> depth_publisher_;
  std::shared_ptr<dai::DataInputQueue> left_image_in_queue_;

};

class SecondaryCamera: public CameraBase
{
public:
  SecondaryCamera(std::shared_ptr<rclcpp::Node> node, std::string id):
    CameraBase(node)
  {
    initialize(id, "left");
    left_image_out_queue_ = device_->getOutputQueue("left_out", 8, false);
  }

  virtual std::shared_ptr<dai::Pipeline> getPipeline() override
  {
    auto pipeline = CameraBase::getPipeline();

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
