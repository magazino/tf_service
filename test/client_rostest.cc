// Copyright 2019 Magazino GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "tf2/exceptions.h"

#include "tf2_ros/buffer_client.h"
#include "tf_service/buffer_client.h"

// Tests both the tf_service and the optional legacy functionality.
// See rostest launch file.
constexpr char kExpectedServerName[] = "/tf_service";
constexpr char kExpectedLegacyServerName[] = "/tf2_buffer_server";
constexpr char kExpectedTargetFrame[] = "map";
constexpr char kExpectedSourceFrame[] = "odom";

class ClientRostest
    : public testing::TestWithParam<std::shared_ptr<tf2_ros::BufferInterface>> {
};

TEST_P(ClientRostest, waitForServerSucceeds) {
  // This only applies to tf_service::BufferClient, so no GetParam() here.
  tf_service::BufferClient buffer(kExpectedServerName);
  EXPECT_TRUE(buffer.waitForServer(ros::Duration(0.1)));
  EXPECT_TRUE(buffer.isConnected());
}

TEST_P(ClientRostest, waitForServerFails) {
  // This only applies to tf_service::BufferClient, so no GetParam() here.
  tf_service::BufferClient buffer("/wrong_server_name");
  EXPECT_FALSE(buffer.waitForServer(ros::Duration(0.1)));
  EXPECT_FALSE(buffer.isConnected());
}

TEST_P(ClientRostest, canTransform) {
  auto buffer = GetParam();
  EXPECT_TRUE(buffer->canTransform(kExpectedTargetFrame, kExpectedSourceFrame,
                                   ros::Time(0), ros::Duration(0.1)));
  EXPECT_FALSE(
      buffer->canTransform("bla", "blub", ros::Time(0), ros::Duration(0.1)));
}

TEST_P(ClientRostest, canTransformAdvanced) {
  auto buffer = GetParam();
  EXPECT_TRUE(buffer->canTransform(kExpectedTargetFrame, ros::Time(0),
                                   kExpectedSourceFrame, ros::Time(0),
                                   kExpectedTargetFrame, ros::Duration(0.1)));
  EXPECT_FALSE(buffer->canTransform("bla", ros::Time(0), "blub", ros::Time(0),
                                    "bla", ros::Duration(0.1)));
}

TEST_P(ClientRostest, lookupTransform) {
  auto buffer = GetParam();
  EXPECT_NO_FATAL_FAILURE(
      buffer->lookupTransform(kExpectedTargetFrame, kExpectedSourceFrame,
                              ros::Time(0), ros::Duration(0.1)));
  EXPECT_THROW(
      buffer->lookupTransform("bla", "blub", ros::Time(0), ros::Duration(0.1)),
      tf2::LookupException);
}

TEST_P(ClientRostest, lookupTransformAdvanced) {
  auto buffer = GetParam();
  EXPECT_NO_FATAL_FAILURE(buffer->lookupTransform(
      kExpectedTargetFrame, ros::Time(0), kExpectedSourceFrame, ros::Time(0),
      kExpectedTargetFrame, ros::Duration(0.1)));
  EXPECT_THROW(buffer->lookupTransform("bla", ros::Time(0), "blub",
                                       ros::Time(0), "bla", ros::Duration(0.1)),
               tf2::LookupException);
}

INSTANTIATE_TEST_CASE_P(
    AllClientRostests, ClientRostest,
    testing::Values(
        std::make_shared<tf_service::BufferClient>(kExpectedServerName),
        std::make_shared<tf2_ros::BufferClient>(kExpectedLegacyServerName)));

int main(int argc, char** argv) {
  ros::init(argc, argv, "client_rostest");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
