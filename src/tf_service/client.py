# Copyright 2019 Magazino GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# For consistency, the docstrings of tf2_ros.BufferInterface methods were copied
# from geometry2/tf2_ros/src/tf2_ros/buffer_client.py, which is
# subject to a BSD License. See 3rdparty/geometry2_LICENSE for details.

import rospy
# Importing tf2_geometry_msgs to register geometry_msgs
# types with tf2_ros.TransformRegistration
import tf2_geometry_msgs
import tf2_ros

from tf_service import client_binding
from tf_service.decorators import translate_exceptions


class BufferClient(tf2_ros.BufferInterface):
    """
    Extends the raw C++ binding to the full Python tf2_ros.BufferInterface,
    adding methods like transform() that don't exist in the C++ interface.
    The interface is exactly the same as the old action-based client.
    """
    @translate_exceptions
    def __init__(self, server_node_name, keepalive_period=None):
        """
        :param server_node_name: name of the tf_service server ROS node
        :param keepalive_period:
            rospy.Duration defining how often to periodically check the
            connection to the server and try to reconnect in the background
            if it dropped
        """
        tf2_ros.BufferInterface.__init__(self)
        # All actual work is done by the C++ binding.
        client_binding.roscpp_init_once()
        self.client = client_binding.BufferClientBinding(server_node_name)
        rospy.on_shutdown(client_binding.roscpp_shutdown)
        # Keepalive timer must be implemented here. It would require a spinner
        # in the C++ binding.
        if keepalive_period is not None:
            self._keepalive_timer = rospy.Timer(keepalive_period,
                                                self._keepalive_callback,
                                                reset=True)

    @translate_exceptions
    def wait_for_server(self, timeout=rospy.Duration(-1)):
        """
        Block until the server is ready to respond to requests and reconnect.

        :param timeout: Time to wait for the server.
        :return: True if the server is ready, false otherwise.
        :rtype: bool
        """
        return self.client.wait_for_server(timeout)

    @translate_exceptions
    def is_connected(self):
        return self.client.is_connected()

    @translate_exceptions
    def reconnect(self, timeout=rospy.Duration(10)):
        return self.client.reconnect(timeout)

    @translate_exceptions
    def async_reconnect(self, timeout=rospy.Duration(10)):
        return self.client.async_reconnect(timeout)

    def _keepalive_callback(self, _):
        if not self.is_connected():
            self.async_reconnect(rospy.Duration(-1))

    @translate_exceptions
    def lookup_transform(self, target_frame, source_frame, time,
                         timeout=rospy.Duration(0.0)):
        """
        Get the transform from the source frame to the target frame.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        return self.client.lookup_transform(target_frame, source_frame, time,
                                            timeout)

    @translate_exceptions
    def lookup_transform_full(self, target_frame, target_time, source_frame,
                              source_time, fixed_frame,
                              timeout=rospy.Duration(0.0)):
        """
        Get the transform from the source frame to the target frame using the advanced API.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        return self.client.lookup_transform(target_frame, target_time,
                                            source_frame, source_time,
                                            fixed_frame, timeout)

    @translate_exceptions
    def can_transform(self, target_frame, source_frame, time,
                      timeout=rospy.Duration(0.0)):
        """
        Check if a transform from the source frame to the target frame is possible.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param return_debug_type: (Optional) If true, return a tuple representing debug information.
        :return: True if the transform is possible, false otherwise.
        :rtype: bool
        """
        return self.client.can_transform(target_frame, source_frame, time,
                                         timeout, "")

    @translate_exceptions
    def can_transform_full(self, target_frame, target_time, source_frame,
                           source_time, fixed_frame,
                           timeout=rospy.Duration(0.0)):
        """
        Check if a transform from the source frame to the target frame is possible (advanced API).

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param return_debug_type: (Optional) If true, return a tuple representing debug information.
        :return: True if the transform is possible, false otherwise.
        :rtype: bool
        """
        return self.client.can_transform(target_frame, target_time,
                                         source_frame, source_time,
                                         fixed_frame, timeout, "")
