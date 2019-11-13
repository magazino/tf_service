#include "ros/ros.h"

#include "simple_tf_buffer_server/buffer_client.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "client_test");
  auto node_handle = std::make_shared<ros::NodeHandle>();

  SimpleBufferClient buffer("/simple_tf_buffer_server/can_transform",
                            "/simple_tf_buffer_server/lookup_transform",
                            node_handle);

  while (ros::ok()) {
    if (buffer.canTransform("map", "map_carto", ros::Time(0),
                            ros::Duration(1))) {
      ROS_INFO_STREAM(buffer.lookupTransform("map", "map_carto", ros::Time(0),
                                             ros::Duration(1)));
    }
    ros::Duration(0.01).sleep();
  }
}
