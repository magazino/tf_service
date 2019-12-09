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

#include <iostream>
#include <memory>

#include "ros/ros.h"
#include "tf_service/buffer_server.h"

#include "boost/program_options.hpp"

namespace po = boost::program_options;

int main(int argc, char** argv) {
  int num_threads = 0;

  po::options_description desc("Options");
  // clang-format off
  desc.add_options()
    ("help", "show usage")
    ("num_threads", po::value<int>(&num_threads)->default_value(10),
     "Number of handler threads. 0 means number of CPU cores.")
  ;
  // clang-format on
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
  } catch (const po::error& exception) {
    std::cerr << exception.what() << std::endl;
    return EXIT_FAILURE;
  }
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return EXIT_FAILURE;
  }

  // boost::po overflows unsigned int for negative values passed to argv,
  // so we use a signed one and check manually.
  if (num_threads < 0) {
    std::cerr << "The number of threads can't be negative." << std::endl;
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "tf_service");
  auto private_node_handle = std::make_shared<ros::NodeHandle>("~");

  ROS_INFO_STREAM("Starting server with " << num_threads << " handler threads");
  tf_service::Server server(private_node_handle);
  ros::AsyncSpinner spinner(num_threads);
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();

  return EXIT_SUCCESS;
}