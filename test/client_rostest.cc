#include "gtest/gtest.h"
#include "tf2/exceptions.h"

#include "simple_tf_buffer_server/buffer_client.h"

// See rostest launch file.
constexpr char kExpectedServerName[] = "/simple_tf_buffer_server";
constexpr char kExpectedTargetFrame[] = "map";
constexpr char kExpectedSourceFrame[] = "odom";

namespace sbs = simple_tf_buffer_server;

TEST(ClientRostest, waitForServerSucceeds) {
  sbs::SimpleBufferClient buffer(kExpectedServerName);
  EXPECT_TRUE(buffer.waitForServer(ros::Duration(0.1)));
  EXPECT_TRUE(buffer.isConnected());
}

TEST(ClientRostest, waitForServerFails) {
  sbs::SimpleBufferClient buffer("/wrong_server_name");
  EXPECT_FALSE(buffer.waitForServer(ros::Duration(0.1)));
  EXPECT_FALSE(buffer.isConnected());
}

TEST(ClientRostest, canTransform) {
  sbs::SimpleBufferClient buffer(kExpectedServerName);
  EXPECT_TRUE(buffer.waitForServer(ros::Duration(0.1)));
  EXPECT_TRUE(buffer.canTransform(kExpectedTargetFrame, kExpectedSourceFrame,
                                  ros::Time(0), ros::Duration(0.1)));
  EXPECT_FALSE(
      buffer.canTransform("bla", "blub", ros::Time(0), ros::Duration(0.1)));
}

TEST(ClientRostest, canTransformAdvanced) {
  sbs::SimpleBufferClient buffer(kExpectedServerName);
  EXPECT_TRUE(buffer.waitForServer(ros::Duration(0.1)));
  EXPECT_TRUE(buffer.canTransform(kExpectedTargetFrame, ros::Time(0),
                                  kExpectedSourceFrame, ros::Time(0),
                                  kExpectedTargetFrame, ros::Duration(0.1)));
  EXPECT_FALSE(buffer.canTransform("bla", ros::Time(0), "blub", ros::Time(0),
                                   "bla", ros::Duration(0.1)));
}

TEST(ClientRostest, lookupTransform) {
  sbs::SimpleBufferClient buffer(kExpectedServerName);
  EXPECT_TRUE(buffer.waitForServer(ros::Duration(0.1)));
  EXPECT_NO_FATAL_FAILURE(
      buffer.lookupTransform(kExpectedTargetFrame, kExpectedSourceFrame,
                             ros::Time(0), ros::Duration(0.1)));
  EXPECT_THROW(
      buffer.lookupTransform("bla", "blub", ros::Time(0), ros::Duration(0.1)),
      tf2::LookupException);
}

TEST(ClientRostest, lookupTransformAdvanced) {
  sbs::SimpleBufferClient buffer(kExpectedServerName);
  EXPECT_TRUE(buffer.waitForServer(ros::Duration(0.1)));
  EXPECT_NO_FATAL_FAILURE(buffer.lookupTransform(
      kExpectedTargetFrame, ros::Time(0), kExpectedSourceFrame, ros::Time(0),
      kExpectedTargetFrame, ros::Duration(0.1)));
  EXPECT_THROW(buffer.lookupTransform("bla", ros::Time(0), "blub", ros::Time(0),
                                      "bla", ros::Duration(0.1)),
               tf2::LookupException);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "client_rostest");
  return RUN_ALL_TESTS();
}
