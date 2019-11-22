#include <memory>

#include "pybind11/pybind11.h"

#include "ros/ros.h"
#include "simple_tf_buffer_server/buffer_client.h"

#include "casts/ros_duration.h"
#include "casts/ros_time.h"
#include "casts/transform_stamped.h"

namespace py = pybind11;
namespace sbs = simple_tf_buffer_server;

// Python module "client", will be <pkg_name>.client after catkin build.
PYBIND11_MODULE(client, m) {
  py::class_<sbs::SimpleBufferClient>(m, "SimpleBufferClientBinding")
      .def(py::init([](const std::string& server_node_name) {
             // TODO
             ros::init(ros::M_string(), "simple_tf_buffer_client_py_internal",
                       ros::init_options::AnonymousName);
             return std::make_unique<sbs::SimpleBufferClient>(
                 server_node_name);
           }),
           /* doc strings for args */
           py::arg("server_node_name"))
      .def("can_transform",
           py::overload_cast<const std::string&, const std::string&,
                             const ros::Time&, ros::Duration, std::string*>(
               &sbs::SimpleBufferClient::canTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("source_frame"), py::arg("time"),
           py::arg("timeout"), py::arg("errstr"))
      .def("can_transform",
           py::overload_cast<const std::string&, const ros::Time&,
                             const std::string&, const ros::Time&,
                             const std::string&, ros::Duration, std::string*>(
               &sbs::SimpleBufferClient::canTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("target_time"),
           py::arg("source_frame"), py::arg("source_time"),
           py::arg("fixed_frame"), py::arg("timeout"), py::arg("errstr"))
      .def("lookup_transform",
           py::overload_cast<const std::string&, const std::string&,
                             const ros::Time&, ros::Duration>(
               &sbs::SimpleBufferClient::lookupTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("source_frame"), py::arg("time"),
           py::arg("timeout"), py::return_value_policy::move)
      .def("lookup_transform",
           py::overload_cast<const std::string&, const ros::Time&,
                             const std::string&, const ros::Time&,
                             const std::string&, ros::Duration>(
               &sbs::SimpleBufferClient::lookupTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("target_time"),
           py::arg("source_frame"), py::arg("source_time"),
           py::arg("fixed_frame"), py::arg("timeout"),
           py::return_value_policy::move)
      .def("is_connected", &sbs::SimpleBufferClient::isConnected)
      .def("reconnect", &sbs::SimpleBufferClient::reconnect,
           /* doc strings for args */
           py::arg("timeout"));

  // Register exception translators to this module.
  // They are evaluated from last registered to first registered,
  // so the base exception has to be registered first:
  static py::exception<tf2::TransformException> TransformExceptionPy(
      m, "TransformException");
  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const tf2::TransformException& e) {
      TransformExceptionPy(e.what());
    }
  });
  static py::exception<tf2::ConnectivityException> ConnectivityExceptionPy(
      m, "ConnectivityException");
  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const tf2::ConnectivityException& e) {
      ConnectivityExceptionPy(e.what());
    }
  });
  static py::exception<tf2::ExtrapolationException> ExtrapolationExceptionPy(
      m, "ExtrapolationException");
  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const tf2::ExtrapolationException& e) {
      ExtrapolationExceptionPy(e.what());
    }
  });
  static py::exception<tf2::InvalidArgumentException>
      InvalidArgumentExceptionPy(m, "InvalidArgumentException");
  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const tf2::InvalidArgumentException& e) {
      InvalidArgumentExceptionPy(e.what());
    }
  });
  static py::exception<tf2::LookupException> LookupExceptionPy(
      m, "LookupException");
  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const tf2::LookupException& e) {
      LookupExceptionPy(e.what());
    }
  });
  static py::exception<tf2::TimeoutException> TimeoutExceptionPy(
      m, "TimeoutException");
  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const tf2::TimeoutException& e) {
      TimeoutExceptionPy(e.what());
    }
  });
}
