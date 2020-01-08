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

#pragma once

#include <memory>
#include <string>

#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "tf_service/CanTransform.h"
#include "tf_service/LookupTransform.h"

namespace tf_service {

struct ServerOptions {
  ros::Duration cache_time = ros::Duration(tf2_ros::Buffer::DEFAULT_CACHE_TIME);
  ros::Duration max_timeout = ros::Duration(10);
  bool debug = false;
};

// Exposes TF lookup as a ROS service.
// Since TF buffer lookups are thread-safe, this class can be used with parallel
// callback handlers.
class Server {
 public:
  Server() = delete;

  Server(const ServerOptions& options);

  bool handleLookupTransform(tf_service::LookupTransformRequest& request,
                             tf_service::LookupTransformResponse& response);

  bool handleCanTransform(tf_service::CanTransformRequest& request,
                          tf_service::CanTransformResponse& response);

 private:
  ServerOptions options_;
  std::vector<ros::ServiceServer> service_servers_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace tf_service
