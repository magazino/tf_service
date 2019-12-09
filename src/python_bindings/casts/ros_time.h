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

#include "ros/ros.h"

#include "pybind11/pybind11.h"

namespace pybind11 {
namespace detail {

// ros::Time <-> rospy.Time
template <>
struct type_caster<ros::Time> {
 public:
  /**
   * This macro establishes the name 'Time' in
   * function signatures and declares a local variable
   * 'value' of type Time
   */
  PYBIND11_TYPE_CASTER(ros::Time, _("Time"));

  /**
   * Conversion part 1 (Python->C++): convert a PyObject into a Time
   * instance or return false upon failure. The second argument
   * indicates whether implicit conversions should be applied.
   */
  bool load(handle src, bool) {
    /* Extract PyObject from handle */
    PyObject* source = src.ptr();

    if (!(PyObject_HasAttrString(source, "secs") &&
          PyObject_HasAttrString(source, "nsecs"))) {
      return false;
    }

    PyObject* secs_tmp = PyNumber_Long(PyObject_GetAttrString(source, "secs"));
    PyObject* nsecs_tmp =
        PyNumber_Long(PyObject_GetAttrString(source, "nsecs"));
    if (!(secs_tmp && nsecs_tmp)) {
      return false;
    }

    value = ros::Time(PyLong_AsLong(secs_tmp), PyLong_AsLong(nsecs_tmp));
    Py_DECREF(secs_tmp);
    Py_DECREF(nsecs_tmp);
    /* Ensure return code was OK (to avoid out-of-range errors etc) */
    return !PyErr_Occurred();
  }

  /**
   * Conversion part 2 (C++ -> Python): convert a Time instance into
   * a Python object. The second and third arguments are used to
   * indicate the return value policy and parent object (for
   * ``return_value_policy::reference_internal``) and are generally
   * ignored by implicit casters.
   */
  static handle cast(ros::Time src, return_value_policy /* policy */,
                     handle /* parent */) {
    // Import rospy.Time class.
    object Time = module::import("rospy").attr("Time");
    // Convert C++ Time object to Python Time object.
    object time = Time(::pybind11::cast(src.sec), ::pybind11::cast(src.nsec));
    return time.release();
  }
};

}  // namespace detail
}  // namespace pybind11