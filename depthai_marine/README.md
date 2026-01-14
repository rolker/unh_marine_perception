# depthai_marine

![Build Status](https://img.shields.io/badge/build-unknown-gray)

**Tools for using Luxonis Oak cameras in marine applications.**

## Overview
This package provides a ROS 2 interface for Luxonis Oak cameras (OAK-D, OAK-1, etc.) specifically tailored for the UNH Marine Autonomy framework. It includes nodes for stereo camera setups and utilities for device management.

## Installation

### Dependencies
-   `depthai`
-   `depthai_bridge`
-   `cv_bridge`
-   `camera_info_manager`

### Building
```bash
colcon build --symlink-install --packages-select depthai_marine
```

## Usage

### Launch Files

#### `wide_stereo_launch.py`
Launches a wide-baseline stereo configuration using two discrete OAK cameras.

```bash
ros2 launch depthai_marine wide_stereo_launch.py
```
**Arguments:**
None (Params are hardcoded in the example, override via params file or modification).

### Utilities
```bash
# List connected OAK devices and their IDs
ros2 run depthai_marine list_devices
```

## Nodes

### `wide_stereo`
Manages two OAK cameras acting as a wide-baseline stereo pair.

#### Publications
| Topic | Type | Description |
|---|---|---|
| `/right/image_raw` | `sensor_msgs/msg/Image` | Raw image from the right camera. |
| `/right/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration data for the right camera. |
| `/left/image_raw` | `sensor_msgs/msg/Image` | Raw image from the left camera. |
| `/left/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration data for the left camera. |

#### Parameters
| Name | Type | Default | Description |
|---|---|---|---|
| `right_camera_id` | `string` | `""` | MXID of the right camera. |
| `left_camera_id` | `string` | `""` | MXID of the left camera. |

### `list_devices`
Console utility to print IDs of all connected DepthAI devices.

## License
Apache License 2.0