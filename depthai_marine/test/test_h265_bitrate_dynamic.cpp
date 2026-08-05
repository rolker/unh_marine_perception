#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "depthai_marine/camera_base.hpp"

using namespace std::chrono_literals;

namespace
{

// Device-free CameraBase: overrides the doRestart() test seam with a counter
// so the dynamic-bitrate scheduling path (validation, coalescing timer,
// h265_enable gate) is observable without a physical OAK. Never calls
// initialize(), mirroring test_h265_params.cpp.
class CountingCamera : public depthai_marine::CameraBase
{
public:
  explicit CountingCamera(std::shared_ptr<rclcpp::Node> node)
  : depthai_marine::CameraBase(node)
  {
  }

  int restart_count() const {return restart_count_;}
  int bitrate_kbps() const {return h265_bitrate_kbps_;}

protected:
  void doRestart() override {++restart_count_;}

private:
  std::atomic<int> restart_count_{0};
};

class DynamicBitrateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_dynamic_bitrate");
    // Declared before enableDynamicBitrate() registers its validation
    // callback — same ordering as SeaSurfaceSegmentation (declare in the
    // node constructor, register in initialize()).
    node_->declare_parameter("h265_bitrate_kbps", 4000);
    camera_ = std::make_shared<CountingCamera>(node_);
  }

  // Spin long enough for the 100 ms coalescing timer to fire (with margin).
  void spinFor(std::chrono::milliseconds duration)
  {
    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(node_);
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      exe.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<CountingCamera> camera_;
};

TEST(ValidateBitrateKbps, Bounds)
{
  using depthai_marine::CameraBase;
  EXPECT_FALSE(CameraBase::validateBitrateKbps(0));
  EXPECT_FALSE(CameraBase::validateBitrateKbps(-1));
  EXPECT_FALSE(
    CameraBase::validateBitrateKbps(
      static_cast<int64_t>(std::numeric_limits<int>::max()) + 1));
  EXPECT_TRUE(CameraBase::validateBitrateKbps(1));
  EXPECT_TRUE(CameraBase::validateBitrateKbps(4000));
  EXPECT_TRUE(CameraBase::validateBitrateKbps(std::numeric_limits<int>::max()));
}

TEST_F(DynamicBitrateTest, RejectsNonPositiveValues)
{
  camera_->enableH265(true);
  camera_->enableDynamicBitrate();

  auto result = node_->set_parameter(rclcpp::Parameter("h265_bitrate_kbps", 0));
  EXPECT_FALSE(result.successful);
  result = node_->set_parameter(rclcpp::Parameter("h265_bitrate_kbps", -100));
  EXPECT_FALSE(result.successful);

  spinFor(300ms);
  EXPECT_EQ(camera_->restart_count(), 0);
  EXPECT_EQ(camera_->bitrate_kbps(), 4000);
}

TEST_F(DynamicBitrateTest, AcceptedChangeSchedulesOneRestart)
{
  camera_->enableH265(true);
  camera_->enableDynamicBitrate();

  auto result = node_->set_parameter(rclcpp::Parameter("h265_bitrate_kbps", 800));
  EXPECT_TRUE(result.successful);

  spinFor(400ms);
  EXPECT_EQ(camera_->restart_count(), 1);
  EXPECT_EQ(camera_->bitrate_kbps(), 800);
}

TEST_F(DynamicBitrateTest, RapidSetsCoalesceToOneRestart)
{
  camera_->enableH265(true);
  camera_->enableDynamicBitrate();

  // Back-to-back sets with no executor spinning in between: each set re-arms
  // the 100 ms one-shot timer, so only one restart fires, at the last value.
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("h265_bitrate_kbps", 700)).successful);
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("h265_bitrate_kbps", 600)).successful);
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("h265_bitrate_kbps", 500)).successful);

  spinFor(400ms);
  EXPECT_EQ(camera_->restart_count(), 1);
  EXPECT_EQ(camera_->bitrate_kbps(), 500);
}

TEST_F(DynamicBitrateTest, DisabledH265StoresValueWithoutRestart)
{
  // h265_enable defaults to false — the value must be stored (it applies if
  // H.265 is enabled later) but no restart scheduled: restarting would blank
  // video + NN for zero encoder benefit.
  camera_->enableDynamicBitrate();

  auto result = node_->set_parameter(rclcpp::Parameter("h265_bitrate_kbps", 900));
  EXPECT_TRUE(result.successful);

  spinFor(300ms);
  EXPECT_EQ(camera_->restart_count(), 0);
  EXPECT_EQ(camera_->bitrate_kbps(), 900);
}

TEST_F(DynamicBitrateTest, EqualValueSetDoesNotRestart)
{
  camera_->enableH265(true);
  camera_->enableDynamicBitrate();

  // Same value as the current member — a restart would blank the stream for
  // no change in encoder output.
  auto result = node_->set_parameter(rclcpp::Parameter("h265_bitrate_kbps", 4000));
  EXPECT_TRUE(result.successful);

  spinFor(300ms);
  EXPECT_EQ(camera_->restart_count(), 0);
  EXPECT_EQ(camera_->bitrate_kbps(), 4000);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
