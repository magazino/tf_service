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

import asyncio
import os
import threading
from typing import Optional

import rospy

# Importing tf2_geometry_msgs to register geometry_msgs
# types with tf2_ros.TransformRegistration
import tf2_geometry_msgs  # pylint: disable=unused-import
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rospy.exceptions import TransportTerminated

from tf_service.exceptions import throw_on_error
from tf_service.persistent_service import PersistentService
from tf_service.srv import (
    CanTransform,
    CanTransformRequest,
    CanTransformResponse,
    LookupTransform,
    LookupTransformRequest,
    LookupTransformResponse,
)

CAN_TRANSFORM_SERVICE_NAME = "can_transform"
LOOKUP_TRANSFORM_SERVICE_NAME = "lookup_transform"
DEFAULT_PING_TIMEOUT = rospy.Duration.from_sec(0.1)

# rospy can throw different exceptions depending on when / how the connection
# to the server is lost:
CONNECTION_LOST_ERRORS = (
    rospy.ROSException,
    rospy.ServiceException,
    TransportTerminated,
    BrokenPipeError,
    AttributeError,
)


class BufferClient(tf2_ros.BufferInterface):
    """
    tf_service buffer client, can be used like any other TF BufferInterface.
    """

    def __init__(
        self,
        server_node_name: str,
        keepalive_period: Optional[rospy.Duration] = None,
        ping_timeout: Optional[rospy.Duration] = DEFAULT_PING_TIMEOUT,
    ) -> None:
        """
        :param server_node_name: name of the tf_service server ROS node
        :param keepalive_period:
            rospy.Duration defining how often to periodically check the
            connection to the server and try to reconnect in the background
            if it dropped
        :param ping_timeout: rospy.Duration defining how long to wait for
            a response from the server before assuming that it's unreachable
        """
        tf2_ros.BufferInterface.__init__(self)

        self._reconnection_mutex = threading.Lock()
        self._ping_timeout = ping_timeout

        # rospy's persistent service seems to be not robust against concurrent requests.
        # We use a mutex to enforce sequential requests on the same TCPROS transport.
        # See also comment in SW-94205.
        self._mutex = threading.Lock()

        can_transform_service_full: str = os.path.join(
            server_node_name, CAN_TRANSFORM_SERVICE_NAME
        )
        lookup_transform_service_full: str = os.path.join(
            server_node_name, LOOKUP_TRANSFORM_SERVICE_NAME
        )

        with self._reconnection_mutex:
            self._can_transform_client = PersistentService(
                can_transform_service_full, CanTransform, self._ping_timeout
            )
            self._lookup_transform_client = PersistentService(
                lookup_transform_service_full, LookupTransform, self._ping_timeout
            )

        if keepalive_period is not None:
            self._keepalive_timer = rospy.Timer(
                keepalive_period, self._keepalive_callback, reset=True
            )

    def wait_for_server(self, timeout: Optional[rospy.Duration] = None) -> bool:
        """
        Block until the server is ready to respond to requests and reconnect.

        :param timeout: Time to wait for the server.
        :return: True if the server is ready, false otherwise.
        :rtype: bool
        """
        if self.is_connected():
            return True
        return self.reconnect(timeout)

    def is_connected(self) -> bool:
        return (
            self._can_transform_client.is_valid()
            and self._lookup_transform_client.is_valid()
        )

    def reconnect(self, timeout: Optional[rospy.Duration] = None) -> bool:
        if self.is_connected():
            return True

        if self._reconnection_mutex.locked():
            rospy.logwarn("Already reconnecting to tf_service server.")
            return False

        with self._reconnection_mutex:
            try:
                rospy.loginfo(
                    "Waiting for %s to become available.",
                    self._can_transform_client.resolved_name,
                )
                self._can_transform_client.wait_for_service(timeout)
            except rospy.ROSInterruptException:
                return False
            except rospy.ROSException as error:
                rospy.logerr("Failed to connect to tf_service server: %s", error)
                return False

            self._can_transform_client = PersistentService(
                self._can_transform_client.resolved_name,
                CanTransform,
                self._ping_timeout,
            )
            self._lookup_transform_client = PersistentService(
                self._lookup_transform_client.resolved_name,
                LookupTransform,
                self._ping_timeout,
            )

        rospy.loginfo(
            "Connected to services %s & %s",
            self._can_transform_client.resolved_name,
            self._lookup_transform_client.resolved_name,
        )
        return True

    async def async_reconnect(self, timeout: Optional[rospy.Duration] = None) -> None:
        if self._reconnection_mutex.locked():
            rospy.logdebug("Already asynchronously reconnecting to server.")
            return
        rospy.loginfo("Asynchronously trying to reconnect to tf_service server.")
        self.reconnect(timeout)

    def _keepalive_callback(self, _) -> None:
        if not self.is_connected():
            asyncio.run(self.async_reconnect())

    def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: rospy.Time,
        timeout: rospy.Duration = rospy.Duration(0.0),
    ) -> TransformStamped:
        """
        Get the transform from the source frame to the target frame.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        request = LookupTransformRequest(
            target_frame=target_frame,
            source_frame=source_frame,
            time=time,
            timeout=timeout,
            advanced=False,
        )

        try:
            with self._mutex:
                response: LookupTransformResponse = self._lookup_transform_client.call(
                    request
                )
                throw_on_error(response.status)
                return response.transform
        except CONNECTION_LOST_ERRORS as error:
            raise tf2_ros.TransformException(
                f"service call to buffer server failed: {error}"
            )

    def lookup_transform_full(
        self,
        target_frame: str,
        target_time: rospy.Time,
        source_frame: str,
        source_time: rospy.Time,
        fixed_frame: str,
        timeout: rospy.Duration = rospy.Duration(0.0),
    ) -> TransformStamped:
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
        request = LookupTransformRequest(
            target_frame=target_frame,
            target_time=target_time,
            source_frame=source_frame,
            source_time=source_time,
            fixed_frame=fixed_frame,
            timeout=timeout,
            advanced=True,
        )

        try:
            with self._mutex:
                response: LookupTransformResponse = self._lookup_transform_client.call(
                    request
                )
                throw_on_error(response.status)
                return response.transform
        except CONNECTION_LOST_ERRORS as error:
            raise tf2_ros.TransformException(
                f"service call to buffer server failed: {error}"
            )

    def can_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: rospy.Time,
        timeout: rospy.Duration = rospy.Duration(0.0),
    ) -> bool:
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
        request = CanTransformRequest(
            target_frame=target_frame,
            source_frame=source_frame,
            time=time,
            timeout=timeout,
            advanced=False,
        )

        try:
            with self._mutex:
                response: CanTransformResponse = self._can_transform_client.call(
                    request
                )
                return response.can_transform
        except CONNECTION_LOST_ERRORS as error:
            raise tf2_ros.TransformException(
                f"service call to buffer server failed: {error}"
            )

    def can_transform_full(
        self,
        target_frame: str,
        target_time: rospy.Time,
        source_frame: str,
        source_time: rospy.Time,
        fixed_frame: str,
        timeout: rospy.Duration = rospy.Duration(0.0),
    ) -> bool:
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
        request = CanTransformRequest(
            target_frame=target_frame,
            target_time=target_time,
            source_frame=source_frame,
            source_time=source_time,
            fixed_frame=fixed_frame,
            timeout=timeout,
            advanced=True,
        )

        try:
            with self._mutex:
                response: CanTransformResponse = self._can_transform_client.call(
                    request
                )
                return response.can_transform
        except CONNECTION_LOST_ERRORS as error:
            raise tf2_ros.TransformException(
                f"service call to buffer server failed: {error}"
            )
