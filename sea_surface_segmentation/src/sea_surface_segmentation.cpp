
#include "rclcpp/rclcpp.hpp"

#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"


#include "depthai/device/Device.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"

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

    camera_ = pipeline_->create<dai::node::Camera>();
    camera_->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
    camera_->setPreviewSize(512, 384);
    camera_->setFps(5);
    camera_->setSize(1280, 720);

    image_manip_ = pipeline_->create<dai::node::ImageManip>();
    image_manip_->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
    image_manip_->initialConfig.setResize(512, 384);

    xlink_camera_ = pipeline_->create<dai::node::XLinkOut>();
    xlink_camera_->setStreamName("camera");

    xlink_neural_network_ = pipeline_->create<dai::node::XLinkOut>();
    xlink_neural_network_->setStreamName("neural_network");

    camera_->preview.link(image_manip_->inputImage);
    image_manip_->out.link(neural_network_->input);
    neural_network_->passthrough.link(xlink_camera_->input);
    neural_network_->out.link(xlink_neural_network_->input);

    int yolo_width = 1280;
    int yolo_height = 704;

    std::string yolo_blob_path;
    declare_parameter("yolo_blob_path", yolo_blob_path);
    yolo_blob_path = get_parameter("yolo_blob_path").as_string();

    if(yolo_blob_path != "")
    {
      float yolo_confidence_threshold = 0.5;
      declare_parameter("yolo_confidence_threshold", yolo_confidence_threshold);
      yolo_confidence_threshold = get_parameter("yolo_confidence_threshold").as_double();
  
      int yolo_num_classes = 2;
      declare_parameter("yolo_number_of_classes", yolo_num_classes);
      yolo_num_classes = get_parameter("yolo_number_of_classes").as_int();
  
      std::vector<double> anchors = {10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319};
      declare_parameter("yolo_anchors", anchors);
      anchors = get_parameter("yolo_anchors").as_double_array();
  
      std::vector<float> anchors_float;
      for(auto anchor: anchors)
      {
        anchors_float.push_back(static_cast<float>(anchor));
      }
  
      std::vector<std::string> anchor_mask_labels;
      declare_parameter("yolo_anchor_mask_labels", anchor_mask_labels);
      anchor_mask_labels = get_parameter("yolo_anchor_mask_labels").as_string_array();
  
      std::map<std::string, std::vector<int64_t>> anchor_masks_64;
  
      for(auto anchor_mask_label: anchor_mask_labels)
      {
        declare_parameter("yolo_anchor_masks."+anchor_mask_label, std::vector<int>());
        anchor_masks_64[anchor_mask_label] = get_parameter("yolo_anchor_masks."+anchor_mask_label).as_integer_array();
      }
  
      std::map<std::string, std::vector<int>> anchor_masks;
      for(auto anchor_mask: anchor_masks_64)
      {
        std::vector<int> anchor_mask_int;
        for(auto mask: anchor_mask.second)
        {
          anchor_mask_int.push_back(static_cast<int>(mask));
        }
        anchor_masks[anchor_mask.first] = anchor_mask_int;
      }
  
  
  
      double iou_threshold = 0.5;
      declare_parameter("yolo_iou_threshold", iou_threshold);
      iou_threshold = get_parameter("yolo_iou_threshold").as_double();
  
      declare_parameter("yolo_width", yolo_width);
      yolo_width = get_parameter("yolo_width").as_int();
      declare_parameter("yolo_height", yolo_height);
      yolo_height = get_parameter("yolo_height").as_int();
  


      detection_network_ = pipeline_->create<dai::node::YoloDetectionNetwork>();
      detection_network_->setConfidenceThreshold(yolo_confidence_threshold);
      detection_network_->setNumClasses(yolo_num_classes);
      detection_network_->setCoordinateSize(4);
      detection_network_->setAnchors(anchors_float);
      detection_network_->setAnchorMasks(anchor_masks);
      detection_network_->setIouThreshold(iou_threshold);
      detection_network_->setBlobPath(yolo_blob_path);
      detection_network_->setNumInferenceThreads(2);
      detection_network_->input.setBlocking(false);

      yolo_image_manip_ = pipeline_->create<dai::node::ImageManip>();
      yolo_image_manip_->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
      yolo_image_manip_->initialConfig.setResize(yolo_width, yolo_height);
      yolo_image_manip_->setMaxOutputFrameSize(yolo_width*yolo_height*3);

      xlink_yolo_ = pipeline_->create<dai::node::XLinkOut>();
      xlink_yolo_->setStreamName("detections");

      xlink_yolo_rgb_ = pipeline_->create<dai::node::XLinkOut>();
      xlink_yolo_rgb_->setStreamName("detections_preview");

      camera_->preview.link(yolo_image_manip_->inputImage);
      yolo_image_manip_->out.link(detection_network_->input);
      detection_network_->out.link(xlink_yolo_->input);
      detection_network_->passthrough.link(xlink_yolo_rgb_->input);
    }


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
      camera_name+"/segmentation/passthrough/image_raw",
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg, image_converter_.get(), std::placeholders::_1, std::placeholders::_2),
      10,
      camera_info,
      camera_name+"/segmentation/passthrough",
      false
    );

    image_publisher_->addPublisherCallback();

    segmentation_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(frame_id_, true);
    segmentation_camera_info_ = segmentation_converter_->calibrationToCameraInfo(calibration_handler, dai::CameraBoardSocket::CAM_A, 128, 96);


    segmentation_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ADatatype> >(
      segmentation_queue_,
      node,
      camera_name+"/segmentation",
      std::bind(&SeaSurfaceSegmentation::segmentationCallback, this, std::placeholders::_1, std::placeholders::_2),
      10,
      segmentation_camera_info_,
      camera_name+"/segmentation",
      false
    );

    segmentation_publisher_->addPublisherCallback();

    if(yolo_blob_path != "")
    {
      auto yolo_color_queue = device_->getOutputQueue("detections_preview", 5, false);
      auto yolo_detection_queue = device_->getOutputQueue("detections", 5, false);

      yolo_image_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(frame_id_, true);
      auto yolo_camera_info = yolo_image_converter_->calibrationToCameraInfo(calibration_handler, dai::CameraBoardSocket::CAM_A, yolo_width, yolo_height);
  
      yolo_image_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> >(
        yolo_color_queue,
        node,
        camera_name+"/detections/passthrough/image_raw",
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, yolo_image_converter_.get(), std::placeholders::_1, std::placeholders::_2),
        10,
        yolo_camera_info,
        camera_name+"/detections/passthrough",
        false
      );
  
      yolo_image_publisher_->addPublisherCallback();

      yolo_detection_converter_ = std::make_shared<dai::rosBridge::ImgDetectionConverter>(frame_id_, yolo_width, yolo_height, false);

      yolo_detection_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<vision_msgs::msg::Detection2DArray, dai::ImgDetections> >(
        yolo_detection_queue,
        node,
        camera_name+"/detections",
        std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg, yolo_detection_converter_.get(), std::placeholders::_1, std::placeholders::_2),
        10,
        yolo_camera_info,
        camera_name,
        false
      );
      yolo_detection_publisher_->addPublisherCallback();

    }

  }

private:

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
  std::shared_ptr<dai::node::YoloDetectionNetwork> detection_network_;
  std::shared_ptr<dai::node::ImageManip> image_manip_;
  std::shared_ptr<dai::node::ImageManip> yolo_image_manip_;
  std::shared_ptr<dai::node::Camera> camera_;
  std::shared_ptr<dai::node::XLinkOut> xlink_camera_;
  std::shared_ptr<dai::node::XLinkOut> xlink_neural_network_;
  std::shared_ptr<dai::node::XLinkOut> xlink_yolo_;
  std::shared_ptr<dai::node::XLinkOut> xlink_yolo_rgb_;
  std::shared_ptr<dai::DataOutputQueue> camera_queue_;
  std::shared_ptr<dai::DataOutputQueue> segmentation_queue_;

  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::rosBridge::ImageConverter> image_converter_;
  std::shared_ptr<dai::rosBridge::ImageConverter> yolo_image_converter_;
  std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> > image_publisher_;
  std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> > yolo_image_publisher_;

  std::shared_ptr<dai::rosBridge::ImageConverter> segmentation_converter_;
  std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ADatatype> > segmentation_publisher_;

  std::shared_ptr<dai::rosBridge::ImgDetectionConverter> yolo_detection_converter_;
  std::shared_ptr<dai::rosBridge::BridgePublisher<vision_msgs::msg::Detection2DArray, dai::ImgDetections> > yolo_detection_publisher_;

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
