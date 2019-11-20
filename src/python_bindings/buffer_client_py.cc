#include <memory>

#include "pybind11/pybind11.h"

#include "ros/ros.h"
#include "simple_tf_buffer_server/buffer_client.h"

#include "casts/ros_duration.h"
#include "casts/ros_time.h"
#include "casts/transform_stamped.h"

namespace py = pybind11;

// Python module "client", will be <pkg_name>.client after catkin build.
PYBIND11_MODULE(client, m) {
  py::class_<tf2_ros::SimpleBufferClient>(m, "SimpleBufferClient")
      .def(py::init([](const std::string& server_node_name) {
             // TODO
             ros::init(ros::M_string(), "simple_tf_buffer_client_py_internal",
                       ros::init_options::AnonymousName);
             return std::make_unique<tf2_ros::SimpleBufferClient>(
                 server_node_name);
           }),
           /* doc strings for args */
           py::arg("server_node_name"))
      .def("can_transform",
           py::overload_cast<const std::string&, const std::string&,
                             const ros::Time&, ros::Duration, std::string*>(
               &tf2_ros::SimpleBufferClient::canTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("source_frame"), py::arg("time"),
           py::arg("timeout"), py::arg("errstr"))
      .def("can_transform",
           py::overload_cast<const std::string&, const ros::Time&,
                             const std::string&, const ros::Time&,
                             const std::string&, ros::Duration, std::string*>(
               &tf2_ros::SimpleBufferClient::canTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("target_time"),
           py::arg("source_frame"), py::arg("source_time"),
           py::arg("fixed_frame"), py::arg("timeout"), py::arg("errstr"))
      .def("lookup_transform",
           py::overload_cast<const std::string&, const std::string&,
                             const ros::Time&, ros::Duration>(
               &tf2_ros::SimpleBufferClient::lookupTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("source_frame"), py::arg("time"),
           py::arg("timeout"), py::return_value_policy::move)
      .def("lookup_transform",
           py::overload_cast<const std::string&, const ros::Time&,
                             const std::string&, const ros::Time&,
                             const std::string&, ros::Duration>(
               &tf2_ros::SimpleBufferClient::lookupTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("target_time"),
           py::arg("source_frame"), py::arg("source_time"),
           py::arg("fixed_frame"), py::arg("timeout"),
           py::return_value_policy::move)
      .def("is_connected", &tf2_ros::SimpleBufferClient::isConnected)
      .def("reconnect", &tf2_ros::SimpleBufferClient::reconnect,
           /* doc strings for args */
           py::arg("timeout"));
}
