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
| `void initialize(std::string id, std::string label, std::string frame_id = "")` | Initializes the connection to the OAK device with the given MXID (`id`) and assigns a logger label. `frame_id` stamps `header.frame_id` on every `Image` / `CameraInfo` / `FFMPEGPacket` from the publishers this method constructs; when empty, defaults to `<label>_optical_frame` (`image_geometry` / REP-103 convention). Pass explicitly for namespaced frames (e.g. `"bizzy/oak_forward_optical"`). |
| `void applyParams(const CameraParams & params)` | Bulk-applies all per-camera settings before `initialize`. See `CameraParams`. |
| `void setPreviewSize(int width, int height)` | Sets the resolution of the camera preview output (default 1280×720). |
| `void setVideoSize(int width, int height)` | Sets the ISP output resolution that feeds both `video` (H.265 encoder input) and `preview` (default 1280×720). |
| `void enableVideo(bool enable)` | Enables or disables the raw `image_raw` host publication (default true). |
| `void enableH265(bool enable)` | Opt in to the on-device H.265/H.264 encoder path. Publishes `FFMPEGPacket` on `<camera>/image_raw/ffmpeg` — see [h265_transport.md](h265_transport.md). |
| `void setH265BitrateKbps(int kbps)` / `setH265KeyframeFrequencyFrames(int)` / `setH265Profile(const std::string &)` | Encoder tuning knobs. |
| `virtual std::shared_ptr<dai::Pipeline> getPipeline()` | Virtual method to construct the DepthAI pipeline. Override this to add more nodes (e.g., neural networks, stereo depth). |

#### Static Methods
| Method | Description |
|---|---|
| `static dai::VideoEncoderProperties::Profile parseProfile(const std::string &)` | Parses `"H265_MAIN"` / `"H264_MAIN"` / `"H264_BASELINE"` / `"H264_HIGH"`. Throws `std::invalid_argument` otherwise. |
| `static std::string profileEncoding(dai::VideoEncoderProperties::Profile)` | Returns the `FFMPEGPacket.encoding` string: `"hevc"` for H.265 profiles, `"h264"` for H.264 profiles. |

### `depthai_marine::CameraParams`
Bundled per-camera configuration populated from ROS params and passed to `CameraBase::applyParams`. Covers preview/video resolution, FPS, and the H.265 encoder knobs. See [h265_transport.md](h265_transport.md) for the full field reference.

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

### `depthai_marine::FFMPEGPublisher`
Codec-agnostic publisher for `ffmpeg_image_transport_msgs/msg/FFMPEGPacket` on
`<topic>/image_raw/ffmpeg`. Carries H.265 or H.264 depending on the profile
configured on the DepthAI `VideoEncoder`. Reads `dai::EncodedFrame` from
`VideoEncoder::out` and converts via `dai::ros::ImageConverter::toRosFFMPEGPacket`
so timestamps share the same ROS-time base offset as the sibling `sensor_msgs/Image`.
QoS is `rclcpp::SensorDataQoS()` to match the `ffmpeg_image_transport` subscriber
convention. Instantiated automatically by `CameraBase::initialize()` when
`h265_enable=true` — see [h265_transport.md](h265_transport.md).

The constructor takes an optional `frame_id` parameter (after `encoding`) that
stamps `header.frame_id` on every published `FFMPEGPacket`. When empty, falls
back to `topic_name` for backwards compatibility; `CameraBase::initialize()`
provides the URDF-aligned `<label>_optical_frame` default for callers that
go through it.

### `depthai_marine::ImagePublisher`
A helper class wrapping `depthai_bridge` to publish images from a DepthAI queue to a ROS 2 topic.

#### Definition
`#include <depthai_marine/image_publisher.hpp>`

#### Public Methods
| Method | Description |
|---|---|
| `ImagePublisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Device> device, std::string queue_name, std::string topic_name, std::string frame_id = "")` | Connects a `DataOutputQueue` from the device to a ROS publisher on `topic_name`. `frame_id` stamps `header.frame_id` on every `Image` / `CameraInfo`; when empty, falls back to `topic_name` for backwards compatibility. Most callers should go through `CameraBase::initialize()`, which provides the URDF-aligned `<label>_optical_frame` default. |

#### Usage Example
```cpp
#include <depthai_marine/image_publisher.hpp>

// Assuming 'device' is available
auto pub = std::make_shared<depthai_marine::ImagePublisher>(node, device, "video", "camera/color");
```
