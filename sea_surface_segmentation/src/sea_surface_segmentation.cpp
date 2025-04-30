
#include "rclcpp/rclcpp.hpp"

#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"


#include "depthai/device/Device.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include "sensor_msgs/msg/image.hpp"

class SeaSurfaceSegmentation: public rclcpp::Node
{
public:
  explicit SeaSurfaceSegmentation()
  : rclcpp::Node("sea_surface_segmentation")
  {
  }

  void initialize()
  {
    ros_base_time_ = get_clock()->now();
    steady_base_time_ = std::chrono::steady_clock::now();
    
    declare_parameter("neural_network", std::string());
    auto blob_path = get_parameter("neural_network").as_string();

    std::string mx_id = "x";
    declare_parameter("mx_id", mx_id);
    mx_id = get_parameter("mx_id").as_string();

    declare_parameter("frame_id", "camera_optical_frame");

    std::string camera_name = "oak";
    declare_parameter("camera_name", camera_name);
    camera_name = get_parameter("camera_name").as_string();

    pipeline_ = std::make_shared<dai::Pipeline>();

    pipeline_->setOpenVINOVersion(dai::OpenVINO::VERSION_2021_4);
    neural_network_ = pipeline_->create<dai::node::NeuralNetwork>();
    neural_network_->setBlobPath(blob_path);
    neural_network_->setNumPoolFrames(4);
    neural_network_->input.setBlocking(false);
    neural_network_->setNumInferenceThreads(2);

    camera_ = pipeline_->create<dai::node::ColorCamera>();
    camera_->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
    camera_->setPreviewSize(512, 384);
    camera_->setInterleaved(false);
    camera_->setFps(5);
    camera_->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);

    xlink_camera_ = pipeline_->create<dai::node::XLinkOut>();
    xlink_camera_->setStreamName("camera");

    xlink_neural_network_ = pipeline_->create<dai::node::XLinkOut>();
    xlink_neural_network_->setStreamName("neural_network");

    camera_->preview.link(neural_network_->input);
    neural_network_->passthrough.link(xlink_camera_->input);
    neural_network_->out.link(xlink_neural_network_->input);

    auto available_devices = dai::Device::getAllAvailableDevices();
    for(auto device_info: available_devices)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Device Mx ID: " << device_info.getMxId());
      if(device_info.getMxId() == mx_id)
      {
        if(device_info.state == X_LINK_UNBOOTED || device_info.state == X_LINK_BOOTLOADER)
        {
          device_ = std::make_shared<dai::Device>(*pipeline_, device_info);
          break;
        }
        else if(device_info.state == X_LINK_BOOTED)
        {
          RCLCPP_ERROR_STREAM(get_logger(), "Device with MxID " << mx_id << " is already booted on a different process.");
          throw std::runtime_error("\" DepthAI Device with MxId  \"" + mx_id + "\" is already booted on different process.  \"");
        }
      }
      else if(mx_id == "x")
      {
        device_ = std::make_shared<dai::Device>(*pipeline_);
      }
    }

    if(!device_)
      throw std::runtime_error("\" DepthAI Device not found.  \"");

    camera_queue_ = device_->getOutputQueue("camera", 5, false);
    segmentation_queue_ = device_->getOutputQueue("neural_network", 5, false);
 
    auto calibration_handler = device_->readCalibration();

    frame_id_ = get_parameter("frame_id").as_string();
    image_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(frame_id_, true);
    auto camera_info = image_converter_->calibrationToCameraInfo(calibration_handler, dai::CameraBoardSocket::CAM_A, 512, 384);


    auto node = shared_from_this();

    image_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> >(
      camera_queue_,
      node,
      camera_name+"/image_raw",
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg, image_converter_.get(), std::placeholders::_1, std::placeholders::_2),
      10,
      camera_info,
      camera_name,
      false
    );

    image_publisher_->addPublisherCallback();

    segmentation_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(frame_id_, true);
    segmentation_camera_info_ = segmentation_converter_->calibrationToCameraInfo(calibration_handler, dai::CameraBoardSocket::CAM_A, 128, 96);


    segmentation_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ADatatype> >(
      segmentation_queue_,
      node,
      camera_name+"/segmentation_raw",
      std::bind(&SeaSurfaceSegmentation::segmentationCallback, this, std::placeholders::_1, std::placeholders::_2),
      10,
      segmentation_camera_info_,
      camera_name,
      false
    );

    segmentation_publisher_->addPublisherCallback();

  }

private:

  //void segmentationCallback(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data)
  void segmentationCallback(std::shared_ptr<dai::ADatatype> data, std::deque<sensor_msgs::msg::Image>& outImageMsgs)
  {
    auto in_det = std::dynamic_pointer_cast<dai::NNData>(data);

    auto layer_data = in_det->getLayerFp16("prediction");

    sensor_msgs::msg::Image image_message;
    image_message.header.frame_id = frame_id_;

    std::chrono::_V2::steady_clock::time_point tstamp = in_det->getTimestamp();

    image_message.header.stamp = dai::ros::getFrameTime(ros_base_time_, steady_base_time_, tstamp);
    image_message.height = 96;
    image_message.width = 128;
    image_message.step = image_message.width*3;
    image_message.encoding = "rgb8";

    auto image_area = image_message.width * image_message.height;

    for(std::size_t i = 0; i < layer_data.size()/3; i++)
    {
      double sum = 0.0;
      for(int j = 0; j < 3; j++)
        sum += exp(layer_data[i+j*image_area]);
      for(int j = 0; j < 3; j++)
        image_message.data.push_back(255*exp(layer_data[i+j*image_area])/sum);
    }

    outImageMsgs.push_back(image_message);
  }


  std::shared_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::node::NeuralNetwork> neural_network_;
  std::shared_ptr<dai::node::ColorCamera> camera_;
  std::shared_ptr<dai::node::XLinkOut> xlink_camera_;
  std::shared_ptr<dai::node::XLinkOut> xlink_neural_network_;
  std::shared_ptr<dai::DataOutputQueue> camera_queue_;
  std::shared_ptr<dai::DataOutputQueue> segmentation_queue_;

  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::rosBridge::ImageConverter> image_converter_;
  std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> > image_publisher_;

  std::shared_ptr<dai::rosBridge::ImageConverter> segmentation_converter_;
  std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ADatatype> > segmentation_publisher_;

  sensor_msgs::msg::CameraInfo segmentation_camera_info_;
  std::string frame_id_;

  rclcpp::Time ros_base_time_;
  std::chrono::time_point<std::chrono::steady_clock> steady_base_time_;


};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto sss = std::make_shared<SeaSurfaceSegmentation>();
  sss->initialize();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sss->get_node_base_interface());
  exe.spin();
  
  rclcpp::shutdown();

  return 0;
}
