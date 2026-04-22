#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "depthai_marine/camera_base.hpp"

using depthai_marine::CameraBase;
using dai::VideoEncoderProperties;

TEST(ParseProfile, KnownNames)
{
  EXPECT_EQ(CameraBase::parseProfile("H265_MAIN"), VideoEncoderProperties::Profile::H265_MAIN);
  EXPECT_EQ(CameraBase::parseProfile("H264_MAIN"), VideoEncoderProperties::Profile::H264_MAIN);
  EXPECT_EQ(CameraBase::parseProfile("H264_BASELINE"), VideoEncoderProperties::Profile::H264_BASELINE);
  EXPECT_EQ(CameraBase::parseProfile("H264_HIGH"), VideoEncoderProperties::Profile::H264_HIGH);
}

TEST(ParseProfile, UnknownNameThrows)
{
  EXPECT_THROW(CameraBase::parseProfile(""), std::invalid_argument);
  EXPECT_THROW(CameraBase::parseProfile("h265_main"), std::invalid_argument);  // case-sensitive
  EXPECT_THROW(CameraBase::parseProfile("MJPEG"), std::invalid_argument);      // not supported
  EXPECT_THROW(CameraBase::parseProfile("AV1"), std::invalid_argument);
}

TEST(ProfileEncoding, H265GivesHevc)
{
  EXPECT_EQ(CameraBase::profileEncoding(VideoEncoderProperties::Profile::H265_MAIN), "hevc");
}

TEST(ProfileEncoding, H264ProfilesGiveH264)
{
  EXPECT_EQ(CameraBase::profileEncoding(VideoEncoderProperties::Profile::H264_MAIN), "h264");
  EXPECT_EQ(CameraBase::profileEncoding(VideoEncoderProperties::Profile::H264_BASELINE), "h264");
  EXPECT_EQ(CameraBase::profileEncoding(VideoEncoderProperties::Profile::H264_HIGH), "h264");
}

TEST(ProfileEncoding, UnsupportedProfileThrows)
{
  // MJPEG is a valid DepthAI profile but has no FFMPEGPacket.encoding mapping
  // — the H.265 publisher is not the right path for JPEG.
  EXPECT_THROW(
    CameraBase::profileEncoding(VideoEncoderProperties::Profile::MJPEG),
    std::invalid_argument);
}

TEST(CameraParamsDefaults, MatchPlanValues)
{
  // Guards against accidental default changes. Plan ISSUE-4 specifies
  // these defaults; any change here should be intentional and documented
  // in docs/h265_transport.md.
  depthai_marine::CameraParams p;
  EXPECT_EQ(p.h265_enable, false);
  EXPECT_EQ(p.h265_bitrate_kbps, 4000);
  EXPECT_EQ(p.h265_keyframe_frequency_frames, 30);
  EXPECT_EQ(p.h265_profile, "H265_MAIN");
  EXPECT_EQ(p.video_width, 1280);
  EXPECT_EQ(p.video_height, 720);
  EXPECT_EQ(p.enable_video, true);
}

TEST(PipelineAssembly, BaselineWithoutH265)
{
  // Pipeline graph assembly is a pure C++ operation; no device needed.
  // Baseline case: default params, h265_enable=false — single preview
  // XLinkOut branch.
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("pipeline_assembly_baseline_test");
  CameraBase base(node);
  ASSERT_NO_THROW(base.getPipeline());
  rclcpp::shutdown();
}

TEST(PipelineAssembly, WithH265Enabled)
{
  // Exercises the new VideoEncoder → XLinkOut("out") branch without
  // connecting a device. Catches silent DepthAI API breakage (e.g. a
  // renamed Profile enum or removed Encoder::out output) at CI time
  // instead of at platform rollout.
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("pipeline_assembly_h265_test");
  CameraBase base(node);
  depthai_marine::CameraParams params;
  params.h265_enable = true;
  base.applyParams(params);
  ASSERT_NO_THROW(base.getPipeline());
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
