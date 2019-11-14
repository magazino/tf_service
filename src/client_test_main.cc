#include "ros/ros.h"

#include "simple_tf_buffer_server/buffer_client.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "client_test");
  auto node_handle = std::make_shared<ros::NodeHandle>();

  tf2_ros::SimpleBufferClient buffer("/simple_tf_buffer_server", node_handle);

  while (ros::ok()) {
    std::string errstr;
    if (buffer.canTransform("map", "map_carto", ros::Time(0), ros::Duration(1),
                            &errstr)) {
      buffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(1));
    } else {
      ROS_INFO_STREAM(errstr);
    }
    ros::Duration(0.01).sleep();
  }
}
