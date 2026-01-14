# depthai_marine API Reference

## Classes

### `depthai_marine::CameraBase`
Base class for creating DepthAI camera nodes in the UNH Marine Autonomy framework. It handles the connection to the OAK device, pipeline creation, and basic image publishing.

#### Definition
`#include <depthai_marine/camera_base.hpp>`

#### Public Methods
| Method | Description |
|---|---|
| `CameraBase(std::shared_ptr<rclcpp::Node> node)` | Constructor. Requires a ROS 2 node handle. |
| `void initialize(std::string id, std::string label)` | Initializes the connection to the OAK device with the given MXID (`id`) and assigns a logger label. |
| `void setPreviewSize(int width, int height)` | Sets the resolution of the camera preview stream (default 1280x720). |
| `void enableVideo(bool enable)` | Enables or disables the video stream (default true). |
| `virtual std::shared_ptr<dai::Pipeline> getPipeline()` | Virtual method to construct the DepthAI pipeline. Override this to add more nodes (e.g., neural networks, stereo depth). |

#### Usage Example
```cpp
#include <depthai_marine/camera_base.hpp>
#include <rclcpp/rclcpp.hpp>

class MyCamera : public depthai_marine::CameraBase
{
public:
  MyCamera(std::shared_ptr<rclcpp::Node> node) : CameraBase(node) {
    // Custom pipeline setup can go here or in getPipeline()
  }
};

// Inside a node:
auto cam = std::make_shared<MyCamera>(node);
cam->initialize("MXID...", "my_camera");
```

### `depthai_marine::ImagePublisher`
A helper class wrapping `depthai_bridge` to publish images from a DepthAI queue to a ROS 2 topic.

#### Definition
`#include <depthai_marine/image_publisher.hpp>`

#### Public Methods
| Method | Description |
|---|---|
| `ImagePublisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Device> device, std::string queue_name, std::string topic_name)` | Connects a `DataOutputQueue` from the device to a ROS publisher on `topic_name`. |

#### Usage Example
```cpp
#include <depthai_marine/image_publisher.hpp>

// Assuming 'device' is available
auto pub = std::make_shared<depthai_marine::ImagePublisher>(node, device, "video", "camera/color");
```
