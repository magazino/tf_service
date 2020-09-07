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

#include <functional>

#include "tf_service/buffer_server.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TF2Error.h"

#include "tf_service/constants.h"

namespace tf_service {

Server::Server(const ServerOptions& options)
    : options_(options),
      tf_buffer_(options_.cache_time, options_.debug),
      tf_listener_(tf_buffer_) {
  tf_buffer_.setUsingDedicatedThread(true);
  ros::NodeHandle private_node_handle("~");
  service_servers_.push_back(private_node_handle.advertiseService(
      kLookupTransformServiceName, &Server::handleLookupTransform, this));
  service_servers_.push_back(private_node_handle.advertiseService(
      kCanTransformServiceName, &Server::handleCanTransform, this));
  if (options_.add_legacy_server) {
    optional_legacy_server_ = std::make_unique<tf2_ros::BufferServer>(
        std::cref(tf_buffer_), options_.legacy_server_namespace,
        false /* auto_start */);
    optional_legacy_server_->start();
  }
}

bool Server::handleLookupTransform(
    tf_service::LookupTransformRequest& request,
    tf_service::LookupTransformResponse& response) {
  // Make sure not to block forever if someone sends a long timeout.
  if (request.timeout > options_.max_timeout) {
    response.status.error = tf2_msgs::TF2Error::INVALID_ARGUMENT_ERROR;
    response.status.error_string =
        "Server is configured to block requests with a timeout above " +
        std::to_string(options_.max_timeout.toSec()) + " seconds.";
    return true;
  }
  geometry_msgs::TransformStamped transform;
  try {
    if (request.advanced) {
      transform = tf_buffer_.lookupTransform(
          request.target_frame, request.target_time, request.source_frame,
          request.source_time, request.fixed_frame, request.timeout);
    } else {
      transform =
          tf_buffer_.lookupTransform(request.target_frame, request.source_frame,
                                     request.time, request.timeout);
    }
  } catch (const tf2::ConnectivityException& exception) {
    response.status.error = tf2_msgs::TF2Error::CONNECTIVITY_ERROR;
    response.status.error_string = exception.what();
    return true;
  } catch (const tf2::ExtrapolationException& exception) {
    response.status.error = tf2_msgs::TF2Error::EXTRAPOLATION_ERROR;
    response.status.error_string = exception.what();
    return true;
  } catch (const tf2::InvalidArgumentException& exception) {
    response.status.error = tf2_msgs::TF2Error::INVALID_ARGUMENT_ERROR;
    response.status.error_string = exception.what();
    return true;
  } catch (const tf2::LookupException& exception) {
    response.status.error = tf2_msgs::TF2Error::LOOKUP_ERROR;
    response.status.error_string = exception.what();
    return true;
  } catch (const tf2::TimeoutException& exception) {
    response.status.error = tf2_msgs::TF2Error::TIMEOUT_ERROR;
    response.status.error_string = exception.what();
    return true;
  } catch (const tf2::TransformException& exception) {
    response.status.error = tf2_msgs::TF2Error::TRANSFORM_ERROR;
    response.status.error_string = exception.what();
    return true;
  }
  response.status.error = tf2_msgs::TF2Error::NO_ERROR;
  response.status.error_string = "Success.";
  response.transform = transform;
  return true;
}

bool Server::handleCanTransform(tf_service::CanTransformRequest& request,
                                tf_service::CanTransformResponse& response) {
  if (request.timeout > options_.max_timeout) {
    response.can_transform = false;
    response.errstr =
        "Server is configured to block requests with a timeout above " +
        std::to_string(options_.max_timeout.toSec()) + " seconds.";
    return true;
  }
  if (request.advanced) {
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

}  // namespace tf_service
