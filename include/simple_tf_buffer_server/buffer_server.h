#include <memory>
#include <string>

#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "simple_tf_buffer_server/CanTransform.h"
#include "simple_tf_buffer_server/LookupTransform.h"

namespace tf2_ros {

// Exposes TF lookup as a ROS service.
// Since TF buffer lookups are thread-safe, this class can be used with parallel
// callback handlers.
class SimpleBufferServer {
 public:
  SimpleBufferServer() = delete;

  SimpleBufferServer(std::shared_ptr<ros::NodeHandle> private_node_handle);

  bool HandleLookupTransform(
      simple_tf_buffer_server::LookupTransformRequest& request,
      simple_tf_buffer_server::LookupTransformResponse& response);

  bool HandleCanTransform(
      simple_tf_buffer_server::CanTransformRequest& request,
      simple_tf_buffer_server::CanTransformResponse& response);

 private:
  std::shared_ptr<ros::NodeHandle> private_node_handle_;
  std::vector<ros::ServiceServer> service_servers_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace tf2_ros
