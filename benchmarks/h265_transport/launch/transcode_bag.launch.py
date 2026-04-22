"""Transcode a CompressedImage topic from a rosbag2 MCAP bag into an
FFMPEGPacket topic using ffmpeg_image_transport (H.265, HW-mimic flags) and
record the result to a new bag.

Pipeline:
    bag --play--> <in>/compressed
           --republish compressed->raw--> /h265_bench/raw
           --republish raw->ffmpeg--> /h265_bench/encoded/ffmpeg
           --record--> <output_bag>

libx265 is constrained to what the OAK Myriad X hardware VideoEncoder can do
(Main profile, no B-frames, single ref, no look-ahead, no adaptive quant,
fixed-GOP CBR). See benchmarks/h265_transport/README.md for sources.

Usage:
    ros2 launch benchmarks/h265_transport/launch/transcode_bag.launch.py \\
        bag:=~/data/logs/bizzy_images/bag_2026-04-21T13.58.31 \\
        input_topic:=/bizzy/sensors/cameras/oak_forward/image_raw \\
        output_bag:=/tmp/out_forward_b1500k_g15 \\
        bitrate:=1500000 \\
        gop_size:=15

Shutdown happens automatically when `ros2 bag play` exits.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag = LaunchConfiguration("bag")
    input_topic = LaunchConfiguration("input_topic")
    output_bag = LaunchConfiguration("output_bag")
    bitrate = LaunchConfiguration("bitrate")
    gop_size = LaunchConfiguration("gop_size")
    x265_level = LaunchConfiguration("x265_level")
    play_rate = LaunchConfiguration("play_rate")
    raw_topic = LaunchConfiguration("raw_topic")
    encoded_base = LaunchConfiguration("encoded_base")

    args = [
        DeclareLaunchArgument("bag", description="Path to input rosbag2 bag (dir or .mcap)"),
        DeclareLaunchArgument(
            "input_topic",
            description=(
                "Base topic of the CompressedImage stream to transcode "
                "(e.g. /bizzy/sensors/cameras/oak_forward/image_raw — the "
                "bag's /compressed will be appended automatically)"
            ),
        ),
        DeclareLaunchArgument(
            "output_bag",
            description="Output bag path (must not exist); recorded as MCAP",
        ),
        DeclareLaunchArgument(
            "bitrate",
            default_value="1500000",
            description="Target CBR bitrate in bits/sec",
        ),
        DeclareLaunchArgument(
            "gop_size",
            default_value="15",
            description="Keyframe interval in frames",
        ),
        DeclareLaunchArgument(
            "x265_level",
            default_value="4.1",
            description="HEVC level-idc (4.1 fits 720p30; raise for larger)",
        ),
        DeclareLaunchArgument(
            "play_rate",
            default_value="1.0",
            description="ros2 bag play rate multiplier (1.0 = real time)",
        ),
        DeclareLaunchArgument(
            "raw_topic",
            default_value="/h265_bench/raw",
            description="Intermediate raw Image topic",
        ),
        DeclareLaunchArgument(
            "encoded_base",
            default_value="/h265_bench/encoded",
            description="Base topic for encoded output (publishes on <base>/ffmpeg)",
        ),
    ]

    x265_params = [
        "bframes=0:ref=1:",
        "rc-lookahead=0:cutree=0:lookahead-slices=0:b-adapt=0:",
        "scenecut=0:keyint=", gop_size, ":min-keyint=", gop_size, ":no-open-gop=1:",
        "aq-mode=0:rect=0:amp=0:",
        "rc=cbr:strict-cbr=1:",
        "bitrate=", bitrate, ":vbv-maxrate=", bitrate, ":vbv-bufsize=", bitrate, ":",
        "level-idc=", x265_level,
    ]

    encoder_av_options = [
        "preset:ultrafast,",
        "tune:zerolatency,",
        "profile:main,",
        "x265-params:", *x265_params,
    ]

    jpeg_decoder = Node(
        package="image_transport",
        executable="republish",
        name="jpeg_decoder",
        arguments=["compressed", "raw"],
        remappings=[
            ("in/compressed", [input_topic, "/compressed"]),
            ("out", raw_topic),
        ],
        output="screen",
    )

    h265_encoder = Node(
        package="image_transport",
        executable="republish",
        name="h265_encoder",
        arguments=["raw", "ffmpeg"],
        remappings=[
            ("in", raw_topic),
            ("out", encoded_base),
        ],
        parameters=[
            {
                "out.ffmpeg.encoder": "libx265",
                "out.ffmpeg.pixel_format": "nv12",
                "out.ffmpeg.gop_size": gop_size,
                "out.ffmpeg.bit_rate": bitrate,
                "out.ffmpeg.encoder_av_options": encoder_av_options,
            }
        ],
        output="screen",
    )

    bag_player = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play",
            bag,
            "--rate", play_rate,
            "--topics", [input_topic, "/compressed"],
        ],
        output="screen",
    )

    bag_recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "-s", "mcap",
            "-o", output_bag,
            [encoded_base, "/ffmpeg"],
        ],
        output="screen",
    )

    shutdown_on_play_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player,
            on_exit=[
                LogInfo(msg="Bag playback finished — shutting down."),
                EmitEvent(event=Shutdown()),
            ],
        )
    )

    return LaunchDescription(
        [
            *args,
            jpeg_decoder,
            h265_encoder,
            bag_recorder,
            bag_player,
            shutdown_on_play_exit,
        ]
    )
