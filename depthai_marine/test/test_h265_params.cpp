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
  auto node = std::make_shared<rclcpp::Node>("pipeline_assembly_baseline_test");
  CameraBase base(node);
  EXPECT_NO_THROW(base.getPipeline());
}

TEST(PipelineAssembly, WithH265Enabled)
{
  // Exercises the new VideoEncoder::out → XLinkOut("h265") branch without
  // connecting a device. Catches silent DepthAI API breakage (e.g. a
  // renamed Profile enum or removed Encoder::out output) at CI time
  // instead of at platform rollout.
  auto node = std::make_shared<rclcpp::Node>("pipeline_assembly_h265_test");
  CameraBase base(node);
  depthai_marine::CameraParams params;
  params.h265_enable = true;
  base.applyParams(params);
  EXPECT_NO_THROW(base.getPipeline());
}

TEST(CameraBaseValidation, SetVideoSizeRejectsNonPositive)
{
  auto node = std::make_shared<rclcpp::Node>("validate_video_size_test");
  CameraBase base(node);
  EXPECT_THROW(base.setVideoSize(0, 720), std::invalid_argument);
  EXPECT_THROW(base.setVideoSize(1280, 0), std::invalid_argument);
  EXPECT_THROW(base.setVideoSize(-1, 720), std::invalid_argument);
  EXPECT_THROW(base.setVideoSize(1280, -1), std::invalid_argument);
  EXPECT_NO_THROW(base.setVideoSize(1280, 720));
}

TEST(CameraBaseValidation, SetH265BitrateKbpsRejectsNonPositive)
{
  auto node = std::make_shared<rclcpp::Node>("validate_bitrate_test");
  CameraBase base(node);
  EXPECT_THROW(base.setH265BitrateKbps(0), std::invalid_argument);
  EXPECT_THROW(base.setH265BitrateKbps(-1), std::invalid_argument);
  EXPECT_NO_THROW(base.setH265BitrateKbps(4000));
}

TEST(CameraBaseValidation, SetH265KeyframeFrequencyFramesRejectsNonPositive)
{
  auto node = std::make_shared<rclcpp::Node>("validate_keyframe_test");
  CameraBase base(node);
  EXPECT_THROW(base.setH265KeyframeFrequencyFrames(0), std::invalid_argument);
  EXPECT_THROW(base.setH265KeyframeFrequencyFrames(-1), std::invalid_argument);
  EXPECT_NO_THROW(base.setH265KeyframeFrequencyFrames(30));
}

TEST(ApplyParams, RoundTripH265Pipeline)
{
  // Round-trip: a non-default CameraParams should drive applyParams() and
  // emerge through getPipeline() without throwing. Catches the case where
  // a future setter starts validating in a way the others don't.
  auto node = std::make_shared<rclcpp::Node>("applyparams_roundtrip_h265_test");
  CameraBase base(node);
  depthai_marine::CameraParams params;
  params.preview_width = 640;
  params.preview_height = 360;
  params.video_width = 1920;
  params.video_height = 1080;
  params.fps = 10.0f;
  params.enable_video = true;
  params.h265_enable = true;
  params.h265_bitrate_kbps = 800;
  params.h265_keyframe_frequency_frames = 60;
  params.h265_profile = "H264_HIGH";
  EXPECT_NO_THROW(base.applyParams(params));
  EXPECT_NO_THROW(base.getPipeline());
}

TEST(ApplyParams, InvalidVideoSizeFromParamsThrows)
{
  // applyParams should propagate the per-setter validation. Any param
  // that fails its own validator must fail applyParams().
  auto node = std::make_shared<rclcpp::Node>("applyparams_invalid_video_test");
  CameraBase base(node);
  depthai_marine::CameraParams params;
  params.video_width = 0;  // setVideoSize rejects this
  EXPECT_THROW(base.applyParams(params), std::invalid_argument);
}

TEST(SetH265Profile, InvalidProfileFailsLazilyAtPipelineAssembly)
{
  // setH265Profile is intentionally lazy — the string is only validated
  // when getPipeline() / initialize() needs to map it to a DepthAI
  // Profile enum. This test pins that contract: an invalid profile is
  // accepted by the setter but rejected at pipeline assembly when h265
  // is enabled. Keeps a future "validate eagerly" change from silently
  // changing the failure point on platforms.
  auto node = std::make_shared<rclcpp::Node>("setprofile_lazy_test");
  CameraBase base(node);
  EXPECT_NO_THROW(base.setH265Profile("NOT_A_REAL_PROFILE"));
  base.enableH265(true);
  EXPECT_THROW(base.getPipeline(), std::invalid_argument);
}

TEST(SetH265Profile, InvalidProfileIsBenignWhenH265Disabled)
{
  // The lazy validation only fires when h265 is enabled. With h265 off,
  // a stale/garbage profile string must not break pipeline assembly —
  // BizzyBoat ships some cameras with h265_enable=False, and those
  // shouldn't trip on a profile string they never use.
  auto node = std::make_shared<rclcpp::Node>("setprofile_disabled_test");
  CameraBase base(node);
  base.setH265Profile("NOT_A_REAL_PROFILE");
  base.enableH265(false);
  EXPECT_NO_THROW(base.getPipeline());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
