// Copyright 2019 Magazino GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tf_service/buffer_client.h"

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

BufferClient::BufferClient(const std::string& server_node_name,
                           const bool use_cache, const ros::Duration cache_time)
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
  if (use_cache) {
    cache_ = std::make_unique<tf2::BufferCore>(cache_time);
  }
}

BufferClient::~BufferClient() { node_handle_.shutdown(); }

bool BufferClient::reconnect(ros::Duration timeout) {
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

bool BufferClient::isConnected() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return (can_transform_client_.isValid() &&
          lookup_transform_client_.isValid());
}

bool BufferClient::waitForServer(const ros::Duration timeout) {
  return isConnected() ? true : reconnect(timeout);
}

void BufferClient::clearCache() {
  if (cache_) {
    cache_->clear();
  }
}

geometry_msgs::TransformStamped BufferClient::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const ros::Time& time, const ros::Duration timeout) const {
  if (cache_) {
    try {
      return cache_->lookupTransform(target_frame, source_frame, time);
    } catch (const tf2::TransformException& exception) {
      // Cache miss, proceed with service call.
    }
  }
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
  if (cache_) {
    cache_->setTransform(srv.response.transform, "unused_authority");
  }
  return srv.response.transform;
}

geometry_msgs::TransformStamped BufferClient::lookupTransform(
    const std::string& target_frame, const ros::Time& target_time,
    const std::string& source_frame, const ros::Time& source_time,
    const std::string& fixed_frame, const ros::Duration timeout) const {
  if (cache_) {
    try {
      return cache_->lookupTransform(target_frame, target_time, source_frame,
                                     source_time, fixed_frame);
    } catch (const tf2::TransformException& exception) {
      // Cache miss, proceed with service call.
    }
  }
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
  if (cache_) {
    cache_->setTransform(srv.response.transform, "unused_authority");
  }
  return srv.response.transform;
}

bool BufferClient::canTransform(const std::string& target_frame,
                                const std::string& source_frame,
                                const ros::Time& time,
                                const ros::Duration timeout,
                                std::string* errstr) const {
  if (cache_) {
    try {
      return cache_->canTransform(target_frame, source_frame, time, errstr);
    } catch (const tf2::TransformException& exception) {
      // Cache miss, proceed with service call.
    }
  }
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

bool BufferClient::canTransform(const std::string& target_frame,
                                const ros::Time& target_time,
                                const std::string& source_frame,
                                const ros::Time& source_time,
                                const std::string& fixed_frame,
                                const ros::Duration timeout,
                                std::string* errstr) const {
  if (cache_) {
    try {
      return cache_->canTransform(target_frame, target_time, source_frame,
                                  source_time, fixed_frame, errstr);
    } catch (const tf2::TransformException& exception) {
      // Cache miss, proceed with service call.
    }
  }
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
