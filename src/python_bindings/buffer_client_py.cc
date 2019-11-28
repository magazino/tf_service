#include <signal.h>
#include <memory>

#include "pybind11/pybind11.h"
#include "ros/ros.h"

#include "casts/ros_duration.h"
#include "casts/ros_time.h"
#include "casts/transform_stamped.h"
#include "tf_service/buffer_client.h"

namespace py = pybind11;
namespace tfs = tf_service;

// Shuts down both roscpp and rospy. ¯\_(ツ)_/¯
void ros_shutdown(int sig) {
  ROS_DEBUG("Shutting down roscpp and rospy.");
  ros::requestShutdown();
  py::module::import("rospy").attr("signal_shutdown")("shutdown request");
  ROS_INFO("bla");
}

// Wires up an internal node that allows us to create node handles.
// Python defers signal handling until the next instruction is handled,
// so it would be blocked by a long C++ call. Here, we can catch this directly
// by installing a signal handler.
static void ros_init_once() {
  if (ros::isInitialized()) {
    return;
  }
  auto init_option_bitflags =
      (ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  ros::init(ros::M_string(), "simple_tf_buffer_client_py_internal",
            init_option_bitflags);
  signal(SIGINT, ros_shutdown);
}

// Python module "client", will be <pkg_name>.client after catkin build.
PYBIND11_MODULE(client_binding, m) {
  py::class_<tfs::BufferClient>(m, "BufferClientBinding")
      .def(py::init([](const std::string& server_node_name) {
             ros_init_once();
             return std::make_unique<tfs::BufferClient>(server_node_name);
           }),
           /* doc strings for args */
           py::arg("server_node_name"))
      .def("can_transform",
           py::overload_cast<const std::string&, const std::string&,
                             const ros::Time&, ros::Duration, std::string*>(
               &tfs::BufferClient::canTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("source_frame"), py::arg("time"),
           py::arg("timeout"), py::arg("errstr"))
      .def("can_transform",
           py::overload_cast<const std::string&, const ros::Time&,
                             const std::string&, const ros::Time&,
                             const std::string&, ros::Duration, std::string*>(
               &tfs::BufferClient::canTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("target_time"),
           py::arg("source_frame"), py::arg("source_time"),
           py::arg("fixed_frame"), py::arg("timeout"), py::arg("errstr"))
      .def("lookup_transform",
           py::overload_cast<const std::string&, const std::string&,
                             const ros::Time&, ros::Duration>(
               &tfs::BufferClient::lookupTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("source_frame"), py::arg("time"),
           py::arg("timeout"), py::return_value_policy::move)
      .def("lookup_transform",
           py::overload_cast<const std::string&, const ros::Time&,
                             const std::string&, const ros::Time&,
                             const std::string&, ros::Duration>(
               &tfs::BufferClient::lookupTransform, py::const_),
           /* doc strings for args */
           py::arg("target_frame"), py::arg("target_time"),
           py::arg("source_frame"), py::arg("source_time"),
           py::arg("fixed_frame"), py::arg("timeout"),
           py::return_value_policy::move)
      .def("is_connected", &tfs::BufferClient::isConnected)
      .def("reconnect", &tfs::BufferClient::reconnect,
           /* doc strings for args */
           py::arg("timeout"))
      .def("wait_for_server", &tfs::BufferClient::waitForServer,
           /* doc strings for args */
           py::arg("timeout"));

  m.def("ros_shutdown", &ros_shutdown);

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
