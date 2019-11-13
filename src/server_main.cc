#include <memory>

#include "ros/ros.h"
#include "simple_tf_buffer_server/buffer_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_tf_buffer_server");
  auto private_node_handle = std::make_shared<ros::NodeHandle>("~");

  SimpleBufferServer server(private_node_handle);

  auto FLAGS_num_threads = 10;
  ros::AsyncSpinner spinner(FLAGS_num_threads);
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();

  return EXIT_SUCCESS;
}