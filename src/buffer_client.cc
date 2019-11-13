#include "simple_tf_buffer_server/buffer_client.h"

#include "tf2/exceptions.h"

#include "simple_tf_buffer_server/CanTransform.h"
#include "simple_tf_buffer_server/LookupTransform.h"
#include "simple_tf_buffer_server/StatusCode.h"

using simple_tf_buffer_server::StatusCode;

SimpleBufferClient::SimpleBufferClient(
    const std::string& can_transform_service_name,
    const std::string& lookup_transform_service_name,
    std::shared_ptr<ros::NodeHandle> node_handle)
    : node_handle_(node_handle) {
  can_transform_client_ =
      node_handle_->serviceClient<simple_tf_buffer_server::CanTransform>(
          can_transform_service_name, true /* persistent */);
  lookup_transform_client_ =
      node_handle_->serviceClient<simple_tf_buffer_server::LookupTransform>(
          lookup_transform_service_name, true /* persistent */);
}

SimpleBufferClient::~SimpleBufferClient() {}

geometry_msgs::TransformStamped SimpleBufferClient::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const ros::Time& time, const ros::Duration timeout) const {
  simple_tf_buffer_server::LookupTransform srv;
  srv.request.target_frame = target_frame;
  srv.request.source_frame = source_frame;
  srv.request.time = time;
  srv.request.timeout = timeout;
  if (!const_cast<ros::ServiceClient*>(&lookup_transform_client_)->call(srv)) {
    throw tf2::TransformException("service call to buffer server failed");
  }
  if (srv.response.status.code == StatusCode::DEADLINE_EXCEEDED) {
    throw tf2::TimeoutException(srv.response.status.message);
  } else if (srv.response.status.code == StatusCode::NOT_FOUND) {
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
  if (!const_cast<ros::ServiceClient*>(&lookup_transform_client_)->call(srv)) {
    throw tf2::TransformException("service call to buffer server failed");
  }
  if (srv.response.status.code == StatusCode::DEADLINE_EXCEEDED) {
    throw tf2::TimeoutException(srv.response.status.message);
  } else if (srv.response.status.code == StatusCode::NOT_FOUND) {
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
  if (!const_cast<ros::ServiceClient*>(&can_transform_client_)->call(srv)) {
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
  if (!const_cast<ros::ServiceClient*>(&can_transform_client_)->call(srv)) {
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
