#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"


#include "vision_msgs/msg/detection2_d_array.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

#include <iostream>

struct HostImageBuffer
{
  std::queue<std::shared_ptr<dai::ImgFrame>> buffer;

  void add(std::shared_ptr<dai::ImgFrame> img)
  {
    if(buffer.size() > 10)
      buffer.pop();
    buffer.push(img);
  }

  std::shared_ptr<dai::ImgFrame> get(int64_t sequence_number)
  {
    while(!buffer.empty())
    {
      auto img = buffer.front();
      if (img->getSequenceNum() < sequence_number)
      {
        buffer.pop();
      }
      else if (img->getSequenceNum() == sequence_number)
      {
        return img;
      }
      else
      {
        return nullptr;
      }
    }
    return nullptr;
  }
};


std::shared_ptr<dai::node::Camera> addCamera(dai::Pipeline &pipeline)
{
    auto camera = pipeline.create<dai::node::Camera>();
    camera->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
    camera->setPreviewSize(1280, 720);
    camera->setSize(1280, 720);
    camera->setFps(2);


    auto camera_xlink_out = pipeline.create<dai::node::XLinkOut>();
    camera_xlink_out->setStreamName("camera");
    camera_xlink_out->input.setBlocking(false);

    // auto frames_xlink_out = pipeline.create<dai::node::XLinkOut>();
    // frames_xlink_out->setStreamName("frames");
    // frames_xlink_out->input.setBlocking(false);

    camera->preview.link(camera_xlink_out->input);
    // camera->preview.link(frames_xlink_out->input);

    return camera;
}

void addFindTextNN(dai::Pipeline &pipeline, std::shared_ptr<dai::node::Camera> camera, std::string blob_path)
{
  auto image_manip = pipeline.create<dai::node::ImageManip>();
  image_manip->initialConfig.setResize(256, 256);
  image_manip->initialConfig.setKeepAspectRatio(false);
  image_manip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
  camera->preview.link(image_manip->inputImage);

  auto find_text_nn = pipeline.create<dai::node::NeuralNetwork>();
  find_text_nn->setBlobPath(blob_path);
  image_manip->out.link(find_text_nn->input);
  find_text_nn->setNumInferenceThreads(2);
  find_text_nn->input.setBlocking(false);

  auto  xlink_find_text_nn_out = pipeline.create<dai::node::XLinkOut>();
  xlink_find_text_nn_out->setStreamName("detections");
  find_text_nn->out.link(xlink_find_text_nn_out->input);

  auto passthrough_xlink_out = pipeline.create<dai::node::XLinkOut>();
  passthrough_xlink_out->setStreamName("detections_passthrough");
  find_text_nn->passthrough.link(passthrough_xlink_out->input);
}

void addTextRecognitionNN(dai::Pipeline &pipeline, std::string blob_path)
{
    auto input_manip = pipeline.create<dai::node::ImageManip>();
    input_manip->inputConfig.setWaitForMessage(true);

    auto input_manip_xlink_image = pipeline.create<dai::node::XLinkIn>();
    input_manip_xlink_image->setStreamName("nn_manip_image");
    input_manip_xlink_image->out.link(input_manip->inputImage);

    auto input_manip_xlink_config = pipeline.create<dai::node::XLinkIn>();
    input_manip_xlink_config->setStreamName("nn_manip_config");
    input_manip_xlink_config->out.link(input_manip->inputConfig);

    auto input_manip_xlink_out = pipeline.create<dai::node::XLinkOut>();
    input_manip_xlink_out->setStreamName("nn_manip_out");
    input_manip->out.link(input_manip_xlink_out->input);

    auto text_recognition_nn = pipeline.create<dai::node::NeuralNetwork>();
    text_recognition_nn->setBlobPath(blob_path);
    text_recognition_nn->setNumInferenceThreads(2);
    input_manip->out.link(text_recognition_nn->input);
    text_recognition_nn->input.setBlocking(true);

    auto text_recognition_nn_xlink_out = pipeline.create<dai::node::XLinkOut>();
    text_recognition_nn_xlink_out->setStreamName("recognitions");
    text_recognition_nn->out.link(text_recognition_nn_xlink_out->input);
}

class MeasureTiming
{
public:
  MeasureTiming(std::shared_ptr<rclcpp::Node> node)
  : logger_(node->get_logger())
  {
    ros_base_time_ = node->get_clock()->now();
    steady_base_time_ = std::chrono::steady_clock::now();

    dai::Pipeline pipeline;
    pipeline.setOpenVINOVersion(dai::OpenVINO::VERSION_2021_4);

    scale_.first = 1280.0/256.0;
    scale_.second = 720.0/256.0;

    auto camera = addCamera(pipeline);

    if(!node->has_parameter("find_text_neural_network"))
      node->declare_parameter("find_text_neural_network", std::string());
    auto find_text_blob_path = node->get_parameter("find_text_neural_network").as_string();

    addFindTextNN(pipeline, camera, find_text_blob_path);


    if(!node->has_parameter("text_recognition_neural_network"))
      node->declare_parameter("text_recognition_neural_network", std::string());
    auto text_recognition_blob_path = node->get_parameter("text_recognition_neural_network").as_string();

    addTextRecognitionNN(pipeline, text_recognition_blob_path);
    
    if(!node->has_parameter("device_id"))
      node->declare_parameter("device_id", std::string(""));
    auto device_id = node->get_parameter("device_id").as_string();

    if(device_id == "")
    {
      device_ = std::make_shared<dai::Device>(pipeline);
      device_id = device_->getDeviceInfo().getMxId();
    }
    else
    {
      device_ = std::make_shared<dai::Device>(pipeline, dai::DeviceInfo(device_id));
    }

    RCLCPP_INFO_STREAM(logger_, "Connected to device: " <<  device_->getDeviceInfo().toString());

    camera_queue_ = device_->getOutputQueue("camera", 5, false);
    //frames_queue_ = device_->getOutputQueue("frames", 5, false);

    find_text_nn_queue_ = device_->getOutputQueue("detections", 1, false);
    find_text_passthrough_queue_ = device_->getOutputQueue("detections_passthrough", 2, false);

    nn_manip_image_queue_ = device_->getInputQueue("nn_manip_image");
    nn_manip_config_queue_ = device_->getInputQueue("nn_manip_config");
    nn_manip_out_queue_ = device_->getOutputQueue("nn_manip_out", 4, false);

    text_recognition_nn_queue_ = device_->getOutputQueue("recognitions", 2, false);

    auto calibration_handler = device_->readCalibration();
   
    if(!node->has_parameter("frame_id"))
      node->declare_parameter("frame_id", "camera_optical_frame");
    frame_id_ = node->get_parameter("frame_id").as_string();

    image_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(frame_id_, true);

    auto camera_info = image_converter_->calibrationToCameraInfo(calibration_handler, dai::CameraBoardSocket::CAM_A, 1280, 720);

    image_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> >(
      camera_queue_,
      node,
      "camera_"+device_id+"/image_raw",
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg, image_converter_.get(), std::placeholders::_1, std::placeholders::_2),
      10,
      camera_info,
      "camera_"+device_id,
      false
    );

    image_publisher_->addPublisherCallback();

    text_detection_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<vision_msgs::msg::Detection2DArray, dai::ADatatype> >(
      find_text_nn_queue_,
      node,
      "camera_"+device_id+"/find_text",
      std::bind(&MeasureTiming::findTextCallback, this, std::placeholders::_1, std::placeholders::_2),
      10,
      "",
      "",
      false
    );

    text_detection_publisher_->addPublisherCallback();

    time_reference_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::TimeReference, dai::ADatatype> >(
      text_recognition_nn_queue_,
      node,
      "camera_"+device_id+"/time_reference",
      std::bind(&MeasureTiming::recognitionCallback, this, std::placeholders::_1, std::placeholders::_2),
      10,
      "",
      "",
      false
    );

    time_reference_publisher_->addPublisherCallback();


    nn_manip_out_image_converter_ = std::make_shared<dai::rosBridge::ImageConverter>(frame_id_, true);
    auto nn_manip_camera_info = nn_manip_out_image_converter_->calibrationToCameraInfo(calibration_handler, dai::CameraBoardSocket::CAM_A, 1280, 720);

    nn_manip_out_image_publisher_ = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> >(
      nn_manip_out_queue_,
      node,
      "camera_"+device_id+"/nn_manip_out",
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg, nn_manip_out_image_converter_.get(), std::placeholders::_1, std::placeholders::_2),
      10,
      nn_manip_camera_info,
      "camera_"+device_id,
      false
    );

    nn_manip_out_image_publisher_->addPublisherCallback();

  }

private:
  void findTextCallback(std::shared_ptr<dai::ADatatype> data, std::deque<vision_msgs::msg::Detection2DArray>& outTextMsgs)
  {
    auto det = std::dynamic_pointer_cast<dai::NNData>(data);

    vision_msgs::msg::Detection2DArray msg;
    std::chrono::_V2::steady_clock::time_point tstamp = det->getTimestamp();
    msg.header.stamp = dai::ros::getFrameTime(ros_base_time_, steady_base_time_, tstamp);
    if(det == nullptr)
      msg.header.frame_id = "none";
    else
    {
      msg.header.frame_id = frame_id_;
      auto layer_names = det->getAllLayerNames();
      auto scores = det->getLayerFp16(layer_names[0]);
      auto geom1 = det->getLayerFp16(layer_names[1]);
      auto geom2 = det->getLayerFp16(layer_names[2]);
      for(auto i = 0; i < 64; i++)
      {
        for(auto j = 0; j < 64; j++)
        {
          auto idx = i*64 + j;
          if(scores[idx] > 0.5)
          {
            auto offsets = std::make_pair(j*4, i*4);
            int steps = 64*64;
            float b0 = geom1[steps*0+idx]; // min y?
            float b1 = geom1[steps*1+idx]; // max x?
            float b2 = geom1[steps*2+idx]; // max y?
            float b3 = geom1[steps*3+idx]; // min x?

            auto cos_a = cos(geom2[idx]);
            auto sin_a = sin(geom2[idx]);

            auto height = b0 + b2;
            auto width = b1 + b3;

            auto offset_x = offsets.first + b1 * cos_a + b2 * sin_a;
            auto offset_y = offsets.second - b1 * sin_a + b2 * cos_a;

            auto p1_x = -sin_a*height + offset_x;
            auto p1_y = -cos_a*height + offset_y;

            auto p3_x = -cos_a*width + offset_x;
            auto p3_y = sin_a*width + offset_y;

            auto center_x = (p1_x + p3_x)/2.0;
            auto center_y = (p1_y + p3_y)/2.0;

            vision_msgs::msg::Detection2D d;
            d.bbox.center.theta = geom2[idx];
            d.bbox.center.position.x = center_x*scale_.first;
            d.bbox.center.position.y = center_y*scale_.second;
            d.bbox.size_x = width*scale_.first;
            d.bbox.size_y = height*scale_.second;
            d.results.resize(1);
            d.results[0].hypothesis.score = scores[idx];
            std::stringstream ss;
            ss << i << "," << j;
            d.id = ss.str();
            msg.detections.push_back(d);
          }
        }
      }
      msg.detections = non_max_suppression(msg.detections, 0.3);

      if(msg.detections.size() > 0)
        outTextMsgs.push_back(msg);

      auto frames = find_text_passthrough_queue_->tryGetAll<dai::ImgFrame>();
      for(auto& frame: frames)
      {
        if(frame)
        {
          host_image_buffer_.add(frame);
        }
      }

      // send frame and detections back to device

      auto frame = host_image_buffer_.get(det->getSequenceNum());
      if(frame)
      {
        bool first = true;
        for(const auto& detection: msg.detections)
        {
          dai::RotatedRect rect;
          rect.center.x = detection.bbox.center.position.x/scale_.first;
          rect.center.y = detection.bbox.center.position.y/scale_.second;
          rect.size.width = detection.bbox.size_x/scale_.first;
          rect.size.height = detection.bbox.size_y/scale_.second;
          rect.angle = -detection.bbox.center.theta*180.0/M_PI;
          dai::ImageManipConfig config;
          config.setCropRotatedRect(rect, false);
          config.setResize(120, 32);
          if(first)
          {
            RCLCPP_INFO_STREAM(logger_, "Sending image for seq num " << frame->getSequenceNum() << " timestamp " << std::fixed << std::setprecision(9) << dai::ros::getFrameTime(ros_base_time_, steady_base_time_, frame->getTimestamp()).nanoseconds()/1000000000.0 << " s");

            auto cv_frame = frame->getCvFrame();
            std::vector<cv::Mat> planes;
            cv::split(cv_frame, planes);

            auto plane_size = planes[0].total()*planes[0].elemSize();
            std::vector<uint8_t> data(3*plane_size);
            std::memcpy(data.data(), planes[0].data, plane_size);
            std::memcpy(data.data()+plane_size, planes[1].data, plane_size);
            std::memcpy(data.data()+2*plane_size, planes[2].data, plane_size);


            dai::ImgFrame img;
            img.setData(data);
            img.setType(dai::ImgFrame::Type::BGR888p);
            img.setWidth(frame->getWidth());
            img.setHeight(frame->getHeight());
            img.setSequenceNum(frame->getSequenceNum());
            img.setTimestamp(frame->getTimestamp());

            nn_manip_image_queue_->send(img);
            first = false;
          }
          else
          {
            config.setReusePreviousImage(true);
            //break; // only send first detect for debugging
          }
          nn_manip_config_queue_->send(config);
        }
      }

    }
  }

  double computeIoU(const vision_msgs::msg::BoundingBox2D& box1, const vision_msgs::msg::BoundingBox2D& box2)
  {
    double box1_x1 = box1.center.position.x - box1.size_x/2.0;
    double box1_y1 = box1.center.position.y - box1.size_y/2.0;
    double box1_x2 = box1.center.position.x + box1.size_x/2.0;
    double box1_y2 = box1.center.position.y + box1.size_y/2.0;

    double box2_x1 = box2.center.position.x - box2.size_x/2.0;
    double box2_y1 = box2.center.position.y - box2.size_y/2.0;
    double box2_x2 = box2.center.position.x + box2.size_x/2.0;
    double box2_y2 = box2.center.position.y + box2.size_y/2.0;

    double inter_x1 = std::max(box1_x1, box2_x1);
    double inter_y1 = std::max(box1_y1, box2_y1);
    double inter_x2 = std::min(box1_x2, box2_x2);
    double inter_y2 = std::min(box1_y2, box2_y2);

    double inter_area = std::max(0.0, inter_x2 - inter_x1) * std::max(0.0, inter_y2 - inter_y1);
    double box1_area = (box1_x2 - box1_x1) * (box1_y2 - box1_y1);
    double box2_area = (box2_x2 - box2_x1) * (box2_y2 - box2_y1);

    double iou = inter_area / (box1_area + box2_area - inter_area);
    return iou;
  }

  std::vector<vision_msgs::msg::Detection2D> non_max_suppression(const std::vector<vision_msgs::msg::Detection2D>& input_detections, float iou_threshold)
  {
    std::vector<vision_msgs::msg::Detection2D> output_detections;
    auto detections = input_detections;
    std::sort(detections.begin(), detections.end(), [](const vision_msgs::msg::Detection2D& a, const vision_msgs::msg::Detection2D& b)
    {
      return a.results[0].hypothesis.score > b.results[0].hypothesis.score;
    });

    while(detections.size() > 0)
    {
      output_detections.push_back(detections[0]);
      std::vector<vision_msgs::msg::Detection2D> remaining_detections;
      for(size_t i = 1; i < detections.size(); i++)
      {
        if(computeIoU(detections[0].bbox, detections[i].bbox) < iou_threshold)
        {
          remaining_detections.push_back(detections[i]);
        }
      }
      detections = remaining_detections;
    }
    return output_detections;
  }

  void recognitionCallback(std::shared_ptr<dai::ADatatype> data, std::deque<sensor_msgs::msg::TimeReference>& outTimeReference)  
  {
    auto seq_num = data->getRaw()->sequenceNum;
    if(seq_num != last_seq_num_)
    {
      dai::ros::updateBaseTime(steady_base_time_, ros_base_time_, total_ns_change_);
    }


    auto recognitions = std::dynamic_pointer_cast<dai::NNData>(data);

    auto tstamp = dai::ros::getFrameTime(ros_base_time_, steady_base_time_, recognitions->getTimestamp());
    RCLCPP_INFO_STREAM(logger_, "Seq No: " << data->getRaw()->sequenceNum << " Timestamp: " << std::fixed << std::setprecision(9) << tstamp.nanoseconds()/1000000000.0 << " s");


    auto layer_data =recognitions->getFirstLayerFp16();

    std::string characters = "0123456789abcdefghijklmnopqrstuvwxyz#";

    int possible_characters = characters.size();
    int output_sequence_length = layer_data.size()/possible_characters;

    std::string result;
    std::vector<float> scores;
    std::vector<float> null_scores;

    for(int i = 0; i < output_sequence_length; i++)
    {
      int max_index = 0;
      float max_value = layer_data[i*possible_characters];
      for(int j = 1; j < possible_characters; j++)
      {
        if(layer_data[i*possible_characters + j] > max_value)
        {
          max_value = layer_data[i*possible_characters + j];
          max_index = j;
        }
        if(j==9)
          break;
      }
      if(max_index == possible_characters - 1)
        continue;
      if(max_value < layer_data[i*possible_characters + possible_characters - 1])
        continue;
      result += characters[max_index];
      scores.push_back(max_value);
      null_scores.push_back(layer_data[i*possible_characters + possible_characters - 1]);
    }

    std::stringstream ss;
    ss << "Recognized: " << result << " (";
    for(auto i = 0; i < scores.size(); i++)
    {
      if(i > 0)
        ss << ",";
      ss << scores[i];
      ss << "/" << null_scores[i];
    }
    ss << ")";
    RCLCPP_INFO_STREAM(logger_, "Seq No: " << data->getRaw()->sequenceNum << " Recognized: " << ss.str());

    sensor_msgs::msg::TimeReference msg;
    msg.header.stamp = tstamp;
    msg.header.frame_id = frame_id_;
    msg.source = result;
    outTimeReference.push_back(msg);


  }

  rclcpp::Logger logger_;

  std::pair<double, double> scale_{1.0, 1.0};
  std::string frame_id_;

  HostImageBuffer host_image_buffer_;

  std::shared_ptr<dai::Device> device_;

  std::shared_ptr<dai::DataOutputQueue> find_text_nn_queue_;
  std::shared_ptr<dai::DataOutputQueue> find_text_passthrough_queue_;

  std::shared_ptr<dai::DataInputQueue> nn_manip_image_queue_;
  std::shared_ptr<dai::DataInputQueue> nn_manip_config_queue_;
  std::shared_ptr<dai::DataOutputQueue> nn_manip_out_queue_;

  std::shared_ptr<dai::DataOutputQueue> text_recognition_nn_queue_;

  std::shared_ptr<dai::DataOutputQueue> camera_queue_;
  std::shared_ptr<dai::DataOutputQueue> frames_queue_;

  std::shared_ptr<dai::rosBridge::ImageConverter> image_converter_;
  std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> > image_publisher_;

  std::shared_ptr<dai::rosBridge::ImageConverter> nn_manip_out_image_converter_;
  std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> > nn_manip_out_image_publisher_;

  std::shared_ptr<dai::rosBridge::BridgePublisher<vision_msgs::msg::Detection2DArray, dai::ADatatype> > text_detection_publisher_;

  std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::TimeReference, dai::ADatatype> > time_reference_publisher_;

  uint64_t last_seq_num_{0};

  rclcpp::Time ros_base_time_;
  std::chrono::time_point<std::chrono::steady_clock> steady_base_time_;
  int64_t total_ns_change_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exe;
  auto node = std::make_shared<rclcpp::Node>("measure_timing");
  auto ml = std::make_shared<MeasureTiming>(node);
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
