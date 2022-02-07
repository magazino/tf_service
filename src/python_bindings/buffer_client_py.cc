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

#include <memory>

#include "pybind11/pybind11.h"
#include "ros/ros.h"

#include "casts/ros_duration.h"
#include "casts/ros_time.h"
#include "casts/transform_stamped.h"
#include "tf_service/buffer_client.h"

namespace py = pybind11;
namespace tfs = tf_service;

// Wires up an internal node that allows us to create node handles.
static void ros_init_once() {
  if (ros::isInitialized()) {
    return;
  }
  // Let Python do signal handling. Note: Python defers signal handling until
  // the next instruction is handled, so it would be blocked by a long C++ call.
  auto init_option_bitflags =
      (ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  ros::init(ros::M_string() /* no remapping */, "tf_service_client_py_internal",
            init_option_bitflags);
}

// Python module "client", will be <pkg_name>.client after catkin build.
// TODO: use PYBIND11_MODULE in new versions, see other TODO at end of macro.
PYBIND11_PLUGIN(client_binding) {
  py::module m("client_binding");
  py::class_<tfs::BufferClient>(m, "BufferClientBinding")
      .def(py::init<const std::string&, const bool, const ros::Duration>(),
           /* doc strings for args */
           py::arg("server_node_name"), py::arg("use_cache"),
           py::arg("cache_time"))
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
           py::arg("timeout"))
      .def("clear_cache", &tfs::BufferClient::clearCache);

  // Needs to be called in Python code.
  m.def("roscpp_init_once", &ros_init_once);

  // Intended for use in Python shutdown hooks.
  m.def("roscpp_shutdown", &ros::requestShutdown);

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

  // TODO: obsolete with PYBIND11_MODULE
  return m.ptr();
}
