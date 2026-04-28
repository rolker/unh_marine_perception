#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "frame_id_resolver.hpp"

using sea_surface_segmentation::resolve_frame_ids;

TEST(ResolveFrameIds, EmptyFrameIdsUsesDefaults)
{
  const std::vector<std::string> names = {"oak_forward", "oak_port"};
  const auto resolved = resolve_frame_ids(names, {});
  ASSERT_EQ(resolved.size(), 2u);
  EXPECT_EQ(resolved[0], "oak_forward_optical_frame");
  EXPECT_EQ(resolved[1], "oak_port_optical_frame");
}

TEST(ResolveFrameIds, FullArrayOverridesEachCamera)
{
  const std::vector<std::string> names = {"oak_forward", "oak_port"};
  const std::vector<std::string> overrides = {"bizzy/oak_forward_optical", "bizzy/oak_port_optical"};
  const auto resolved = resolve_frame_ids(names, overrides);
  ASSERT_EQ(resolved.size(), 2u);
  EXPECT_EQ(resolved[0], "bizzy/oak_forward_optical");
  EXPECT_EQ(resolved[1], "bizzy/oak_port_optical");
}

TEST(ResolveFrameIds, EmptyEntryFallsBackToDefault)
{
  // Mixed: per-entry empty string is the documented escape hatch for
  // "default this one, override the rest" — needed when one camera in
  // a multi-camera launch is unmapped.
  const std::vector<std::string> names = {"oak_forward", "oak_port"};
  const std::vector<std::string> overrides = {"bizzy/oak_forward_optical", ""};
  const auto resolved = resolve_frame_ids(names, overrides);
  ASSERT_EQ(resolved.size(), 2u);
  EXPECT_EQ(resolved[0], "bizzy/oak_forward_optical");
  EXPECT_EQ(resolved[1], "oak_port_optical_frame");
}

TEST(ResolveFrameIds, LengthMismatchThrows)
{
  const std::vector<std::string> names = {"oak_forward", "oak_port", "oak_aft"};
  const std::vector<std::string> shorter = {"a", "b"};
  const std::vector<std::string> longer = {"a", "b", "c", "d"};
  EXPECT_THROW(resolve_frame_ids(names, shorter), std::invalid_argument);
  EXPECT_THROW(resolve_frame_ids(names, longer), std::invalid_argument);
}

TEST(ResolveFrameIds, EmptyCameraNamesProducesEmptyResult)
{
  EXPECT_TRUE(resolve_frame_ids({}, {}).empty());
}

TEST(ResolveFrameIds, SingleCameraWithOverride)
{
  const auto resolved = resolve_frame_ids({"oak_forward"}, {"bizzy/oak_forward_optical"});
  ASSERT_EQ(resolved.size(), 1u);
  EXPECT_EQ(resolved[0], "bizzy/oak_forward_optical");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
