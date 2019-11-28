#include "ros/ros.h"

#include "tf_service/buffer_client.h"

#include "boost/program_options.hpp"

namespace tfs = tf_service;
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

  tfs::SimpleBufferClient buffer("/tf_service");

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
