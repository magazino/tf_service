#include <memory>

#include "pybind11/pybind11.h"

#include "ros/ros.h"
#include "simple_tf_buffer_server/buffer_client.h"

#include "casts/ros_duration.h"
#include "casts/ros_time.h"
#include "casts/transform_stamped.h"

namespace py = pybind11;

// py::arg("bla") can be used to inform Python about the names of arguments.

PYBIND11_MODULE(simple_tf_buffer_client_py, m) {
  py::class_<tf2_ros::SimpleBufferClient>(m, "SimpleBufferClient")
      .def(py::init([](const std::string& server_node_name) {
             // TODO
             ros::init(ros::M_string(), "simple_tf_buffer_client_py_internal",
                       ros::init_options::AnonymousName);
             return std::make_unique<tf2_ros::SimpleBufferClient>(
                 server_node_name);
           }),
           py::arg("server_node_name"))
      .def("can_transform",
           py::overload_cast<const std::string&, const std::string&,
                             const ros::Time&, ros::Duration, std::string*>(
               &tf2_ros::SimpleBufferClient::canTransform, py::const_))
      .def("can_transform",
           py::overload_cast<const std::string&, const ros::Time&,
                             const std::string&, const ros::Time&,
                             const std::string&, ros::Duration, std::string*>(
               &tf2_ros::SimpleBufferClient::canTransform, py::const_))
      .def("lookup_transform",
           py::overload_cast<const std::string&, const std::string&,
                             const ros::Time&, ros::Duration>(
               &tf2_ros::SimpleBufferClient::lookupTransform, py::const_),
           py::return_value_policy::move)
      .def("lookup_transform",
           py::overload_cast<const std::string&, const ros::Time&,
                             const std::string&, const ros::Time&,
                             const std::string&, ros::Duration>(
               &tf2_ros::SimpleBufferClient::lookupTransform, py::const_),
           py::return_value_policy::move)
      .def("is_connected", &tf2_ros::SimpleBufferClient::isConnected)
      .def("reconnect", &tf2_ros::SimpleBufferClient::reconnect);
}
