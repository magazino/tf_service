#pragma once

#include "geometry_msgs/Transform.h"

#include "pybind11/pybind11.h"

#include "ros_time.h"

namespace pybind11 {
namespace detail {

// geometry_msgs::TransformStamped <-> geometry_msgs.msg.TransformStamped
template <>
struct type_caster<geometry_msgs::TransformStamped> {
 public:
  /**
   * This macro establishes the name 'TransformStamped' in
   * function signatures and declares a local variable
   * 'value' of type TransformStamped
   */
  PYBIND11_TYPE_CASTER(geometry_msgs::TransformStamped, _("TransformStamped"));

  /**
   * Conversion part 1 (Python->C++): convert a PyObject into a TransformStamped
   * instance or return false upon failure. The second argument
   * indicates whether implicit conversions should be applied.
   */
  bool load(handle src, bool) {
    /* Extract PyObject from handle */
    PyObject* source = src.ptr();

    if (!(PyObject_HasAttrString(source, "header") &&
          PyObject_HasAttrString(source, "child_frame_id") &&
          PyObject_HasAttrString(source, "transform"))) {
      return false;
    }

    // PyObject* secs_tmp = PyNumber_Long(PyObject_GetAttrString(source,
    // "secs")); PyObject* nsecs_tmp =
    //    PyNumber_Long(PyObject_GetAttrString(source, "nsecs"));
    // if (!(secs_tmp && nsecs_tmp)) {
    //  return false;
    //}

    value = geometry_msgs::TransformStamped();
    // Py_DECREF(secs_tmp);
    // Py_DECREF(nsecs_tmp);
    /* Ensure return code was OK (to avoid out-of-range errors etc) */
    return !PyErr_Occurred();
  }

  /**
   * Conversion part 2 (C++ -> Python): convert a TransformStamped instance into
   * a Python object. The second and third arguments are used to
   * indicate the return value policy and parent object (for
   * ``return_value_policy::reference_internal``) and are generally
   * ignored by implicit casters.
   */
  static handle cast(const geometry_msgs::TransformStamped src,
                     return_value_policy policy /* policy */,
                     handle parent /* parent */) {
    // Import rospy.TransformStamped class.
    object TransformStamped =
        module::import("geometry_msgs.msg").attr("TransformStamped");
    // Convert C++ TransformStamped object to Python TransformStamped object.
    object tf = TransformStamped();

    tf.attr("header").attr("seq") = ::pybind11::cast(src.header.seq);
    tf.attr("header").attr("frame_id") = ::pybind11::cast(src.header.frame_id);
    tf.attr("header").attr("stamp") =
        type_caster<ros::Time>().cast(src.header.stamp, policy, parent);
    tf.attr("child_frame_id") = ::pybind11::cast(src.child_frame_id);
    tf.attr("transform").attr("rotation").attr("w") =
        ::pybind11::cast(src.transform.rotation.w);
    tf.attr("transform").attr("rotation").attr("x") =
        ::pybind11::cast(src.transform.rotation.x);
    tf.attr("transform").attr("rotation").attr("y") =
        ::pybind11::cast(src.transform.rotation.y);
    tf.attr("transform").attr("rotation").attr("z") =
        ::pybind11::cast(src.transform.rotation.z);
    tf.attr("transform").attr("translation").attr("x") =
        ::pybind11::cast(src.transform.translation.x);
    tf.attr("transform").attr("translation").attr("y") =
        ::pybind11::cast(src.transform.translation.y);
    tf.attr("transform").attr("translation").attr("z") =
        ::pybind11::cast(src.transform.translation.z);

    // TODO MichaelGrupp: without release(), the refcount goes to 0 and leaks.
    return tf.release();
  }
};

}  // namespace detail
}  // namespace pybind11