
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

#include "frame_id_resolver.hpp"
#include "segmentation_stamp.hpp"

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
    // Pass `frame_id_` (either a per-camera override from the `frame_ids`
    // ROS param or the resolved `<name>_optical_frame` default) through to
    // CameraBase so the sibling video / H.265 publishers stamp messages
    // with the same frame_id as the NN/segmentation output. Without this,
    // only the segmentation Image carried the URDF-aligned frame; video
    // and H.265 packets fell back to the bare `name` (e.g. `oak_forward`),
    // breaking TF lookups for any consumer of those topics.
    initialize(id, name, frame_id_);

    // Capture the ROS<->steady base offset for stamping segmentation frames from
    // their device capture time (#28). deviceFrameStamp() re-anchors ros_base_time_
    // on every frame, so the exact capture instant here is immaterial — but the
    // members must exist before the first segmentation callback fires.
    ros_base_time_ = node->get_clock()->now();
    steady_base_time_ = std::chrono::steady_clock::now();

    if (enable_nn_) {
        segmentation_queue_ = device_->getOutputQueue("neural_network", 5, false);

        auto calibration_handler = device_->readCalibration();

        segmentation_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(frame_id_, true);
        // The published segmentation image is 128x96 (NN output), but the actual
        // pipeline is: camera preview at (params.preview_width x params.preview_height,
        // typically 1280x720, 16:9) → ImageManip stretches to 512x384 (4:3) →
        // NN downsamples to 128x96. Calling calibrationToCameraInfo directly
        // at (128, 96) would give intrinsics that assume an isotropic scaling
        // from the sensor — wrong, because the preview→NN-input step is an
        // anisotropic stretch. Compute the preview-resolution intrinsics from
        // DepthAI's calibration handler, then scale fx/cx by 128/preview_width
        // and fy/cy by 96/preview_height to reflect the squish. Distortion
        // coefficients are left unchanged: they're a small per-pixel correction
        // in normalised image coordinates, and re-deriving them through a non-
        // affine resize is non-trivial; the dominant aspect-ratio error in
        // cell→pixel projection is what we're correcting here.
        auto preview_camera_info = segmentation_converter_->calibrationToCameraInfo(
          calibration_handler, dai::CameraBoardSocket::CAM_A,
          params.preview_width, params.preview_height);
        constexpr int kSegWidth = 128;
        constexpr int kSegHeight = 96;
        const double scale_x = static_cast<double>(kSegWidth) / params.preview_width;
        const double scale_y = static_cast<double>(kSegHeight) / params.preview_height;
        segmentation_camera_info_ = preview_camera_info;
        segmentation_camera_info_.width = kSegWidth;
        segmentation_camera_info_.height = kSegHeight;
        // K (3x3 intrinsic matrix, row-major): scale fx, cx by x; fy, cy by y.
        segmentation_camera_info_.k[0] *= scale_x;  // fx
        segmentation_camera_info_.k[2] *= scale_x;  // cx
        segmentation_camera_info_.k[4] *= scale_y;  // fy
        segmentation_camera_info_.k[5] *= scale_y;  // cy
        // P (3x4 projection matrix): same scaling on the K-equivalent entries.
        // Tx (p[3]) and Ty (p[7]) are 0 for a monocular setup; scaling is a
        // no-op there but kept for correctness if a stereo bridge ever fills
        // them in.
        segmentation_camera_info_.p[0] *= scale_x;
        segmentation_camera_info_.p[2] *= scale_x;
        segmentation_camera_info_.p[3] *= scale_x;
        segmentation_camera_info_.p[5] *= scale_y;
        segmentation_camera_info_.p[6] *= scale_y;
        segmentation_camera_info_.p[7] *= scale_y;

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
        // EXPLICIT stretch: resize the preview into the NN input shape without
        // preserving aspect ratio. DepthAI's setResize default is to keep the
        // aspect ratio (which center-crops or pads), which would silently drop
        // the side portions of a 16:9 preview when fed to a 4:3 NN. Forcing
        // keep_aspect_ratio=false stretches the whole preview into 512x384 so
        // the segmentation covers the full camera FOV — at the cost of
        // anisotropic pixel scaling, which is corrected in the camera_info
        // built below.
        image_manip->initialConfig.setKeepAspectRatio(false);
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

    // Stamp from the source frame's device capture time, NOT now() (#28). now()
    // discarded the ~123.5 ms fixed pipeline latency, so every downstream TF
    // lookup (SeaSurfaceLayer, segments_to_pointcloud, sea_surface_tuner)
    // resolved a stale camera pose and close buoys never marked. BridgePublisher
    // copies this stamp onto the camera_info sibling, and image_transport onto
    // the compressed sibling, so this one stamp corrects the whole group.
    image_message.header.stamp = sea_surface_segmentation::deviceFrameStamp(
      ros_base_time_, steady_base_time_, total_ns_change_, in_det->getTimestamp());

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
  // ROS<->steady base offset for converting NNData device capture timestamps to
  // ROS time (#28). Re-anchored per frame by deviceFrameStamp().
  rclcpp::Time ros_base_time_;
  std::chrono::time_point<std::chrono::steady_clock> steady_base_time_;
  int64_t total_ns_change_ = 0;
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
