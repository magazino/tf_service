#pragma once

#include <memory>
#include <string>

#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "tf_service/CanTransform.h"
#include "tf_service/LookupTransform.h"

namespace tf_service {

// Exposes TF lookup as a ROS service.
// Since TF buffer lookups are thread-safe, this class can be used with parallel
// callback handlers.
class Server {
 public:
  Server() = delete;

  Server(std::shared_ptr<ros::NodeHandle> private_node_handle);

  bool handleLookupTransform(tf_service::LookupTransformRequest& request,
                             tf_service::LookupTransformResponse& response);

  bool handleCanTransform(tf_service::CanTransformRequest& request,
                          tf_service::CanTransformResponse& response);

 private:
  std::shared_ptr<ros::NodeHandle> private_node_handle_;
  std::vector<ros::ServiceServer> service_servers_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace tf_service
