
#include "rclcpp/rclcpp.hpp"

#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "depthai_marine/camera_base.hpp"
#include "depthai_marine/image_publisher.hpp"

#include "sea_surface_segmentation/frame_id_resolver.hpp"

class SegmentorCamera : public depthai_marine::CameraBase
{
public:
  SegmentorCamera(
    std::shared_ptr<rclcpp::Node> node,
    std::string id,
    std::string name,
    const depthai_marine::CameraParams & params,
    bool enable_nn,
    std::string frame_id)
  : depthai_marine::CameraBase(node),
    name_(name),
    frame_id_(std::move(frame_id)),
    enable_nn_(enable_nn)
  {
    applyParams(params);
    initialize(id, name);

    if (enable_nn_) {
        segmentation_queue_ = device_->getOutputQueue("neural_network", 5, false);

        auto calibration_handler = device_->readCalibration();

        segmentation_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(frame_id_, true);
        segmentation_camera_info_ = segmentation_converter_->calibrationToCameraInfo(calibration_handler, dai::CameraBoardSocket::CAM_A, 128, 96);

        segmentation_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ADatatype> >(
          segmentation_queue_,
          node,
          name+"/segmentation",
          std::bind(&SegmentorCamera::segmentationCallback, this, std::placeholders::_1, std::placeholders::_2),
          10,
          segmentation_camera_info_,
          name+"/segmentation",
          false
        );

        segmentation_publisher_->addPublisherCallback();
    }
  }

  virtual std::shared_ptr<dai::Pipeline> getPipeline() override
  {
    // Call base to setup camera_ and optional video stream (using fps_ and preview size)
    auto pipeline = depthai_marine::CameraBase::getPipeline();

    if (enable_nn_) {
        // Re-fetch parameters from the node to ensure we have the correct path
        std::string blob_path;
        if(node_->has_parameter("neural_network")) {
            blob_path = node_->get_parameter("neural_network").as_string();
        } else {
            RCLCPP_WARN(node_->get_logger(), "Parameter 'neural_network' not set!");
        }

        auto neural_network = pipeline->create<dai::node::NeuralNetwork>();
        neural_network->setBlobPath(blob_path);
        neural_network->setNumPoolFrames(4);
        neural_network->input.setBlocking(false);
        neural_network->setNumInferenceThreads(2);

        auto image_manip = pipeline->create<dai::node::ImageManip>();
        image_manip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
        image_manip->initialConfig.setResize(512, 384);

        // Link camera preview to image manip. Camera FPS is already set by base class.
        camera_->preview.link(image_manip->inputImage);
        image_manip->out.link(neural_network->input);

        auto xlink_neural_network = pipeline->create<dai::node::XLinkOut>();
        xlink_neural_network->setStreamName("neural_network");
        neural_network->out.link(xlink_neural_network->input);
    }
    
    return pipeline;
  }

private:
  void segmentationCallback(std::shared_ptr<dai::ADatatype> data, std::deque<sensor_msgs::msg::Image>& outImageMsgs)
  {
    auto in_det = std::dynamic_pointer_cast<dai::NNData>(data);
    auto layer_data = in_det->getLayerFp16("prediction");

    sensor_msgs::msg::Image image_message;
    image_message.header.frame_id = frame_id_;

    image_message.header.stamp = node_->get_clock()->now(); 

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

  std::string name_;
  std::string frame_id_;
  bool enable_nn_;
  std::shared_ptr<dai::DataOutputQueue> segmentation_queue_;
  std::shared_ptr<dai::rosBridge::ImageConverter> segmentation_converter_;
  std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ADatatype> > segmentation_publisher_;
  sensor_msgs::msg::CameraInfo segmentation_camera_info_;
};

class SeaSurfaceSegmentation : public rclcpp::Node
{
public:
  SeaSurfaceSegmentation() : Node("sea_surface_segmentation_node")
  {
    depthai_marine::CameraParams defaults;

    this->declare_parameter("camera_ids", std::vector<std::string>());
    this->declare_parameter("camera_names", std::vector<std::string>());
    // Optional. Either empty (every camera gets the historical default
    // `<camera_name>_optical_frame`) or the same length as camera_names
    // (per-camera override; a per-entry empty string falls back to the
    // historical default for that one camera). Any other length is a
    // configuration error and the node refuses to initialize.
    this->declare_parameter("frame_ids", std::vector<std::string>());

    declare_parameter("neural_network", std::string(""));

    declare_parameter("enable_video", defaults.enable_video);
    declare_parameter("enable_nn", true);
    declare_parameter("preview_width", defaults.preview_width);
    declare_parameter("preview_height", defaults.preview_height);
    declare_parameter("video_width", defaults.video_width);
    declare_parameter("video_height", defaults.video_height);
    declare_parameter("fps", static_cast<double>(defaults.fps));

    declare_parameter("h265_enable", defaults.h265_enable);
    declare_parameter("h265_bitrate_kbps", defaults.h265_bitrate_kbps);
    declare_parameter("h265_keyframe_frequency_frames", defaults.h265_keyframe_frequency_frames);
    declare_parameter("h265_profile", defaults.h265_profile);
  }

  void initialize()
  {
    std::vector<std::string> camera_ids = this->get_parameter("camera_ids").as_string_array();
    std::vector<std::string> camera_names = this->get_parameter("camera_names").as_string_array();
    std::vector<std::string> frame_ids = this->get_parameter("frame_ids").as_string_array();

    if (camera_ids.size() != camera_names.size()) {
        RCLCPP_ERROR(this->get_logger(), "Number of camera IDs and names must match!");
        return;
    }

    std::vector<std::string> resolved_frame_ids;
    try {
      resolved_frame_ids = sea_surface_segmentation::resolve_frame_ids(camera_names, frame_ids);
    } catch (const std::invalid_argument & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      return;
    }

    depthai_marine::CameraParams params;
    params.enable_video = get_parameter("enable_video").as_bool();
    params.preview_width = get_parameter("preview_width").as_int();
    params.preview_height = get_parameter("preview_height").as_int();
    params.video_width = get_parameter("video_width").as_int();
    params.video_height = get_parameter("video_height").as_int();
    params.fps = static_cast<float>(get_parameter("fps").as_double());
    params.h265_enable = get_parameter("h265_enable").as_bool();
    params.h265_bitrate_kbps = get_parameter("h265_bitrate_kbps").as_int();
    params.h265_keyframe_frequency_frames = get_parameter("h265_keyframe_frequency_frames").as_int();
    params.h265_profile = get_parameter("h265_profile").as_string();

    bool enable_nn = get_parameter("enable_nn").as_bool();

    for (size_t i = 0; i < camera_ids.size(); ++i) {
        RCLCPP_INFO(get_logger(),
            "Initializing camera: %s (MxId: %s, frame_id: %s)",
            camera_names[i].c_str(), camera_ids[i].c_str(),
            resolved_frame_ids[i].c_str());
        auto cam = std::make_shared<SegmentorCamera>(
            shared_from_this(),
            camera_ids[i],
            camera_names[i],
            params,
            enable_nn,
            resolved_frame_ids[i]);
        cameras_.push_back(cam);
    }
  }

private:
  std::vector<std::shared_ptr<SegmentorCamera>> cameras_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto sss = std::make_shared<SeaSurfaceSegmentation>();
  sss->initialize();
  
  // Use MultiThreadedExecutor to handle callbacks from multiple cameras efficiently
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(sss->get_node_base_interface());
  exe.spin();
  
  rclcpp::shutdown();

  return 0;
}
