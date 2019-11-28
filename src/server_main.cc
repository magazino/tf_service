#include <iostream>
#include <memory>

#include "ros/ros.h"
#include "tf_service/buffer_server.h"

#include "boost/program_options.hpp"

namespace tfs = tf_service;
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
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "tf_service");
  auto private_node_handle = std::make_shared<ros::NodeHandle>("~");

  ROS_INFO_STREAM("Starting server with " << num_threads << " handler threads");
  tfs::SimpleBufferServer server(private_node_handle);
  ros::AsyncSpinner spinner(num_threads);
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();

  return EXIT_SUCCESS;
}