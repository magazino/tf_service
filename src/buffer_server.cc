#include "simple_tf_buffer_server/buffer_server.h"

#include "geometry_msgs/TransformStamped.h"
#include "simple_tf_buffer_server/ExceptionType.h"
#include "simple_tf_buffer_server/constants.h"

constexpr float kMaxAllowedTimeout = 10.;

using simple_tf_buffer_server::ExceptionType;

namespace tf2_ros {

SimpleBufferServer::SimpleBufferServer(
    std::shared_ptr<ros::NodeHandle> private_node_handle)
    : private_node_handle_(private_node_handle) {
  tf_buffer_.setUsingDedicatedThread(true);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  service_servers_.push_back(private_node_handle_->advertiseService(
      kLookupTransformServiceName, &SimpleBufferServer::handleLookupTransform,
      this));
  service_servers_.push_back(private_node_handle_->advertiseService(
      kCanTransformServiceName, &SimpleBufferServer::handleCanTransform, this));
}

bool SimpleBufferServer::handleLookupTransform(
    simple_tf_buffer_server::LookupTransformRequest& request,
    simple_tf_buffer_server::LookupTransformResponse& response) {
  // TODO make sure not to block forever if someone sends a long timeout.
  // TODO move to client?
  if (request.timeout > ros::Duration(kMaxAllowedTimeout)) {
    response.status.exception_type = ExceptionType::INTERNAL;
    response.status.message = "Requests with a timeout above " +
                              std::to_string(kMaxAllowedTimeout) +
                              " are not supported.";
    return true;
  }
  geometry_msgs::TransformStamped transform;
  try {
    if (!request.fixed_frame.empty()) {
      transform = tf_buffer_.lookupTransform(
          request.target_frame, request.target_time, request.source_frame,
          request.source_time, request.fixed_frame, request.timeout);
    } else {
      transform =
          tf_buffer_.lookupTransform(request.target_frame, request.source_frame,
                                     request.time, request.timeout);
    }
  } catch (const tf2::TimeoutException& exception) {
    response.status.exception_type = ExceptionType::TIMEOUT_EXCEPTION;
    response.status.message = exception.what();
    return true;
  } catch (const tf2::TransformException& exception) {
    response.status.exception_type = ExceptionType::TRANSFORM_EXCEPTION;
    response.status.message = exception.what();
    return true;
  } catch (...) {
    response.status.exception_type = ExceptionType::OTHER;
    response.status.message =
        "unknown exception while handling lookup transform request";
    ROS_ERROR_STREAM(response.status.message);
    return true;
  }
  response.status.exception_type = ExceptionType::NONE;
  response.status.message = "Success.";
  response.transform = transform;
  return true;
}

bool SimpleBufferServer::handleCanTransform(
    simple_tf_buffer_server::CanTransformRequest& request,
    simple_tf_buffer_server::CanTransformResponse& response) {
  if (!request.fixed_frame.empty()) {
    response.can_transform = tf_buffer_.canTransform(
        request.target_frame, request.target_time, request.source_frame,
        request.source_time, request.fixed_frame, request.timeout,
        &response.errstr);
  } else {
    response.can_transform = tf_buffer_.canTransform(
        request.target_frame, request.source_frame, request.time,
        request.timeout, &response.errstr);
  }
  return true;
}

}  // namespace tf2_ros
