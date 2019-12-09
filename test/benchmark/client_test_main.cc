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

#include "ros/ros.h"

#include "tf_service/buffer_client.h"

#include "boost/program_options.hpp"

namespace po = boost::program_options;

int main(int argc, char** argv) {
  float frequency = 1;

  po::options_description desc("Options");
  // clang-format off
  desc.add_options()
    ("help", "show usage")
    ("frequency", po::value<float>(&frequency)->default_value(10.),
     "main loop frequency")
  ;
  // clang-format on
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "client_test");

  tf_service::BufferClient buffer("/tf_service");

  ros::Rate rate(ros::Duration(1. / frequency));
  while (ros::ok()) {
    std::string errstr;
    if (buffer.canTransform("map", "odom", ros::Time(0), ros::Duration(1),
                            &errstr)) {
      try {
        buffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(1));
      } catch (const tf2::TransformException& exception) {
        ROS_ERROR_STREAM_THROTTLE(10, exception.what());
      }
    } else {
      ROS_ERROR_STREAM_THROTTLE(10, errstr);
      if (!buffer.isConnected()) {
        buffer.reconnect();
      }
    }
    rate.sleep();
    ros::spinOnce();
  }
}
