#pragma once

#include "ros/ros.h"

#include "pybind11/pybind11.h"

namespace pybind11 {
namespace detail {

// ros::Duration <-> rospy.Duration
template <>
struct type_caster<ros::Duration> {
 public:
  /**
   * This macro establishes the name 'Duration' in
   * function signatures and declares a local variable
   * 'value' of type Duration
   */
  PYBIND11_TYPE_CASTER(ros::Duration, _("Duration"));

  /**
   * Conversion part 1 (Python->C++): convert a PyObject into a Duration
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

    value = ros::Duration(PyLong_AsLong(secs_tmp), PyLong_AsLong(nsecs_tmp));
    Py_DECREF(secs_tmp);
    Py_DECREF(nsecs_tmp);
    /* Ensure return code was OK (to avoid out-of-range errors etc) */
    return !PyErr_Occurred();
  }

  /**
   * Conversion part 2 (C++ -> Python): convert a Duration instance into
   * a Python object. The second and third arguments are used to
   * indicate the return value policy and parent object (for
   * ``return_value_policy::reference_internal``) and are generally
   * ignored by implicit casters.
   */
  static handle cast(ros::Duration src, return_value_policy /* policy */,
                     handle /* parent */) {
    // Import rospy.Duration class.
    object Duration = module::import("rospy").attr("Duration");
    // Convert C++ Duration object to Python Duration object.
    object duration =
        Duration(::pybind11::cast(src.sec), ::pybind11::cast(src.nsec));
    return duration.release();
  }
};

}  // namespace detail
}  // namespace pybind11