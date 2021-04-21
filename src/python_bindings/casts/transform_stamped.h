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

#include "geometry_msgs/TransformStamped.h"

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
    // TODO: not implemented!!
    return false;
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
    tf.attr("header").attr("stamp") = ::pybind11::cast(src.header.stamp);
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