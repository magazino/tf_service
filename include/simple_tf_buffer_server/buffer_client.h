#include <memory>
#include <string>

#include "ros/ros.h"
#include "tf2_ros/buffer_interface.h"

class SimpleBufferClient : public tf2_ros::BufferInterface {
 public:
  SimpleBufferClient() = delete;

  SimpleBufferClient(const std::string& can_transform_service_name,
                     const std::string& lookup_transform_service_name,
                     std::shared_ptr<ros::NodeHandle> node_handle);

  ~SimpleBufferClient();

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

 private:
  std::shared_ptr<ros::NodeHandle> node_handle_;
  ros::ServiceClient can_transform_client_;
  ros::ServiceClient lookup_transform_client_;
};
