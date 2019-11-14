#include "simple_tf_buffer_server/buffer_client.h"
#include <chrono>

#include "boost/filesystem.hpp"
#include "tf2/exceptions.h"

#include "simple_tf_buffer_server/CanTransform.h"
#include "simple_tf_buffer_server/ExceptionType.h"
#include "simple_tf_buffer_server/LookupTransform.h"
#include "simple_tf_buffer_server/constants.h"

using simple_tf_buffer_server::ExceptionType;

namespace {
std::string join(const std::string& node_name,
                 const std::string& service_name) {
  boost::filesystem::path node(node_name);
  boost::filesystem::path service(service_name);
  return (node / service).string();
}
}  // namespace

namespace tf2_ros {

SimpleBufferClient::SimpleBufferClient(
    const std::string& server_node_name,
    std::shared_ptr<ros::NodeHandle> node_handle)
    : node_handle_(node_handle) {
  const std::string can_transform_service_full =
      join(server_node_name, kCanTransformServiceName);
  can_transform_client_ =
      node_handle_->serviceClient<simple_tf_buffer_server::CanTransform>(
          can_transform_service_full, true /* persistent */);
  const std::string lookup_transform_service_full =
      join(server_node_name, kLookupTransformServiceName);
  lookup_transform_client_ =
      node_handle_->serviceClient<simple_tf_buffer_server::LookupTransform>(
          lookup_transform_service_full, true /* persistent */);
}

SimpleBufferClient::~SimpleBufferClient() {}

bool SimpleBufferClient::reconnect(ros::Duration timeout) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (!can_transform_client_.waitForExistence(timeout)) {
    ROS_ERROR("Failed to reconnect to server.");
    return false;
  }
  can_transform_client_ =
      node_handle_->serviceClient<simple_tf_buffer_server::CanTransform>(
          can_transform_client_.getService(), true /* persistent */);
  lookup_transform_client_ =
      node_handle_->serviceClient<simple_tf_buffer_server::LookupTransform>(
          lookup_transform_client_.getService(), true /* persistent */);
  ROS_INFO("Reconnected to server.");
  return true;
}

bool SimpleBufferClient::isConnected() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return (can_transform_client_.isValid() &&
          lookup_transform_client_.isValid());
}

geometry_msgs::TransformStamped SimpleBufferClient::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const ros::Time& time, const ros::Duration timeout) const {
  simple_tf_buffer_server::LookupTransform srv;
  srv.request.target_frame = target_frame;
  srv.request.source_frame = source_frame;
  srv.request.time = time;
  srv.request.timeout = timeout;
  bool success;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    success = lookup_transform_client_.call(srv);
  }
  if (!success) {
    throw tf2::TransformException("service call to buffer server failed");
  }
  if (srv.response.status.exception_type == ExceptionType::TIMEOUT_EXCEPTION) {
    throw tf2::TimeoutException(srv.response.status.message);
  } else if (srv.response.status.exception_type ==
             ExceptionType::TRANSFORM_EXCEPTION) {
    throw tf2::TransformException(srv.response.status.message);
  } else if (srv.response.status.exception_type != ExceptionType::NONE) {
    // TODO use other type?
    throw tf2::TransformException(srv.response.status.message);
  }
  return srv.response.transform;
}

geometry_msgs::TransformStamped SimpleBufferClient::lookupTransform(
    const std::string& target_frame, const ros::Time& target_time,
    const std::string& source_frame, const ros::Time& source_time,
    const std::string& fixed_frame, const ros::Duration timeout) const {
  simple_tf_buffer_server::LookupTransform srv;
  srv.request.target_frame = target_frame;
  srv.request.target_time = target_time;
  srv.request.source_frame = source_frame;
  srv.request.source_time = source_time;
  srv.request.fixed_frame = fixed_frame;
  srv.request.timeout = timeout;
  bool success;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    success = lookup_transform_client_.call(srv);
  }
  if (!success) {
    throw tf2::TransformException("service call to buffer server failed");
  }
  if (srv.response.status.exception_type == ExceptionType::TIMEOUT_EXCEPTION) {
    throw tf2::TimeoutException(srv.response.status.message);
  } else if (srv.response.status.exception_type ==
             ExceptionType::TRANSFORM_EXCEPTION) {
    throw tf2::TransformException(srv.response.status.message);
  } else if (srv.response.status.exception_type != ExceptionType::NONE) {
    // TODO use other type?
    throw tf2::TransformException(srv.response.status.message);
  }
  return srv.response.transform;
}

bool SimpleBufferClient::canTransform(const std::string& target_frame,
                                      const std::string& source_frame,
                                      const ros::Time& time,
                                      const ros::Duration timeout,
                                      std::string* errstr) const {
  simple_tf_buffer_server::CanTransform srv;
  srv.request.target_frame = target_frame;
  srv.request.source_frame = source_frame;
  srv.request.time = time;
  srv.request.timeout = timeout;
  bool success;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    success = can_transform_client_.call(srv);
  }
  if (!success) {
    if (errstr != NULL) {
      *errstr = "service call to buffer server failed";
    }
    return false;
  }
  if (errstr != NULL) {
    *errstr = srv.response.errstr;
  }
  return srv.response.can_transform;
}

bool SimpleBufferClient::canTransform(const std::string& target_frame,
                                      const ros::Time& target_time,
                                      const std::string& source_frame,
                                      const ros::Time& source_time,
                                      const std::string& fixed_frame,
                                      const ros::Duration timeout,
                                      std::string* errstr) const {
  simple_tf_buffer_server::CanTransform srv;
  srv.request.target_frame = target_frame;
  srv.request.target_time = target_time;
  srv.request.source_frame = source_frame;
  srv.request.source_time = source_time;
  srv.request.fixed_frame = fixed_frame;
  srv.request.timeout = timeout;
  bool success;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    success = can_transform_client_.call(srv);
  }
  if (!success) {
    if (errstr != NULL) {
      *errstr = "service call to buffer server failed";
    }
    return false;
  }
  if (errstr != NULL) {
    *errstr = srv.response.errstr;
  }
  return srv.response.can_transform;
}

}  // namespace tf2_ros
