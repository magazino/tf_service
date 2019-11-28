#include "tf_service/buffer_client.h"

#include <chrono>

#include "boost/filesystem.hpp"
#include "tf2/exceptions.h"
#include "tf2_msgs/TF2Error.h"

#include "tf_service/CanTransform.h"
#include "tf_service/LookupTransform.h"
#include "tf_service/constants.h"

namespace {

std::string join(const std::string& node_name,
                 const std::string& service_name) {
  boost::filesystem::path node(node_name);
  boost::filesystem::path service(service_name);
  return (node / service).string();
}

void throwOnError(tf2_msgs::TF2Error& status) {
  switch (status.error) {
    case tf2_msgs::TF2Error::CONNECTIVITY_ERROR:
      throw tf2::ConnectivityException(status.error_string);
      break;
    case tf2_msgs::TF2Error::EXTRAPOLATION_ERROR:
      throw tf2::ExtrapolationException(status.error_string);
      break;
    case tf2_msgs::TF2Error::INVALID_ARGUMENT_ERROR:
      throw tf2::InvalidArgumentException(status.error_string);
      break;
    case tf2_msgs::TF2Error::LOOKUP_ERROR:
      throw tf2::LookupException(status.error_string);
      break;
    case tf2_msgs::TF2Error::TIMEOUT_ERROR:
      throw tf2::TimeoutException(status.error_string);
      break;
    case tf2_msgs::TF2Error::TRANSFORM_ERROR:
      throw tf2::TransformException(status.error_string);
      break;
    default:
      break;
  }
}

}  // namespace

namespace tf_service {

SimpleBufferClient::SimpleBufferClient(const std::string& server_node_name)
    : node_handle_(ros::NodeHandle()) {
  const std::string can_transform_service_full =
      join(server_node_name, kCanTransformServiceName);
  can_transform_client_ = node_handle_.serviceClient<tf_service::CanTransform>(
      can_transform_service_full, true /* persistent */);
  const std::string lookup_transform_service_full =
      join(server_node_name, kLookupTransformServiceName);
  lookup_transform_client_ =
      node_handle_.serviceClient<tf_service::LookupTransform>(
          lookup_transform_service_full, true /* persistent */);
}

SimpleBufferClient::~SimpleBufferClient() { node_handle_.shutdown(); }

bool SimpleBufferClient::reconnect(ros::Duration timeout) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (!can_transform_client_.waitForExistence(timeout)) {
    ROS_ERROR("Failed to connect to server.");
    return false;
  }
  can_transform_client_ = node_handle_.serviceClient<tf_service::CanTransform>(
      can_transform_client_.getService(), true /* persistent */);
  lookup_transform_client_ =
      node_handle_.serviceClient<tf_service::LookupTransform>(
          lookup_transform_client_.getService(), true /* persistent */);
  ROS_DEBUG_STREAM("Connected to services "
                   << lookup_transform_client_.getService() << " & "
                   << can_transform_client_.getService());
  return true;
}

bool SimpleBufferClient::isConnected() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return (can_transform_client_.isValid() &&
          lookup_transform_client_.isValid());
}

bool SimpleBufferClient::waitForServer(const ros::Duration timeout) {
  return isConnected() ? true : reconnect(timeout);
}

geometry_msgs::TransformStamped SimpleBufferClient::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const ros::Time& time, const ros::Duration timeout) const {
  tf_service::LookupTransform srv;
  srv.request.target_frame = target_frame;
  srv.request.source_frame = source_frame;
  srv.request.time = time;
  srv.request.timeout = timeout;
  srv.request.advanced = false;
  bool success;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    success = lookup_transform_client_.call(srv);
  }
  if (!success) {
    throw tf2::TransformException("service call to buffer server failed");
  }
  throwOnError(srv.response.status);
  return srv.response.transform;
}

geometry_msgs::TransformStamped SimpleBufferClient::lookupTransform(
    const std::string& target_frame, const ros::Time& target_time,
    const std::string& source_frame, const ros::Time& source_time,
    const std::string& fixed_frame, const ros::Duration timeout) const {
  tf_service::LookupTransform srv;
  srv.request.target_frame = target_frame;
  srv.request.target_time = target_time;
  srv.request.source_frame = source_frame;
  srv.request.source_time = source_time;
  srv.request.fixed_frame = fixed_frame;
  srv.request.timeout = timeout;
  srv.request.advanced = true;
  bool success;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    success = lookup_transform_client_.call(srv);
  }
  if (!success) {
    throw tf2::TransformException("service call to buffer server failed");
  }
  throwOnError(srv.response.status);
  return srv.response.transform;
}

bool SimpleBufferClient::canTransform(const std::string& target_frame,
                                      const std::string& source_frame,
                                      const ros::Time& time,
                                      const ros::Duration timeout,
                                      std::string* errstr) const {
  tf_service::CanTransform srv;
  srv.request.target_frame = target_frame;
  srv.request.source_frame = source_frame;
  srv.request.time = time;
  srv.request.timeout = timeout;
  srv.request.advanced = false;
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
  tf_service::CanTransform srv;
  srv.request.target_frame = target_frame;
  srv.request.target_time = target_time;
  srv.request.source_frame = source_frame;
  srv.request.source_time = source_time;
  srv.request.fixed_frame = fixed_frame;
  srv.request.timeout = timeout;
  srv.request.advanced = true;
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

}  // namespace tf_service
