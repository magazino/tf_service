#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "ros/ros.h"
#include "tf2_ros/buffer_interface.h"

namespace tf_service {

class BufferClient : public tf2_ros::BufferInterface {
 public:
  BufferClient() = delete;

  BufferClient(const std::string& server_node_name);

  ~BufferClient();

  geometry_msgs::TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const ros::Time& time, const ros::Duration timeout) const override;

  geometry_msgs::TransformStamped lookupTransform(
      const std::string& target_frame, const ros::Time& target_time,
      const std::string& source_frame, const ros::Time& source_time,
      const std::string& fixed_frame,
      const ros::Duration timeout) const override;

  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame, const ros::Time& time,
                    const ros::Duration timeout,
                    std::string* errstr = NULL) const override;

  bool canTransform(const std::string& target_frame,
                    const ros::Time& target_time,
                    const std::string& source_frame,
                    const ros::Time& source_time,
                    const std::string& fixed_frame, const ros::Duration timeout,
                    std::string* errstr = NULL) const override;

  bool reconnect(ros::Duration timeout = ros::Duration(10));
  bool isConnected() const;
  bool waitForServer(const ros::Duration timeout = ros::Duration(-1));

 private:
  mutable std::mutex mutex_;

  ros::NodeHandle node_handle_;

  // mutable because ServiceClient::call() isn't const.
  mutable ros::ServiceClient can_transform_client_;     // GUARDED_BY(mutex_);
  mutable ros::ServiceClient lookup_transform_client_;  // GUARDED_BY(mutex_);
};

}  // namespace tf_service
