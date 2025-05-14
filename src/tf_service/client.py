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

import rospy
import rospy.core

# Importing tf2_geometry_msgs to register geometry_msgs
# types with tf2_ros.TransformRegistration
import tf2_geometry_msgs  # pylint: disable=unused-import
import tf2_ros
from rospy.exceptions import TransportInitError
from rospy.impl.tcpros_base import TCPROSTransport
from tf2_msgs.msg import TF2Error

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


def _throw_on_error(status: TF2Error) -> None:
    if status.error == TF2Error.CONNECTIVITY_ERROR:
        raise tf2_ros.ConnectivityException(status.error_string)
    if status.error == TF2Error.EXTRAPOLATION_ERROR:
        raise tf2_ros.ExtrapolationException(status.error_string)
    if status.error == TF2Error.INVALID_ARGUMENT_ERROR:
        raise tf2_ros.InvalidArgumentException(status.error_string)
    if status.error == TF2Error.LOOKUP_ERROR:
        raise tf2_ros.LookupException(status.error_string)
    if status.error == TF2Error.TIMEOUT_ERROR:
        raise tf2_ros.TimeoutException(status.error_string)
    if status.error == TF2Error.TRANSFORM_ERROR:
        raise tf2_ros.TransformException(status.error_string)


def _init_transport(proxy: rospy.ServiceProxy) -> bool:
    """
    Transport initialization of persistent rospy service is deferred to first
    time call() is called.
    This function allows to establish the transport independently of call().
    """
    if proxy.transport is not None:
        return True

    try:
        service_uri = proxy._get_service_uri(proxy.request_class())
        dest_addr, dest_port = rospy.core.parse_rosrpc_uri(service_uri)
    except rospy.ServiceException as error:
        rospy.logerr(str(error))
        return False

    try:
        transport = TCPROSTransport(proxy.protocol, proxy.resolved_name)
        transport.buff_size = proxy.buff_size
        transport.connect(dest_addr, dest_port, service_uri)
    except TransportInitError as error:
        rospy.logerr(str(error))
        return False

    return True


def _is_valid(proxy: rospy.ServiceProxy) -> bool:
    """
    rospy doesn't have an equivalent of ros::ServiceClient::isValid().
    This should do the trick instead...
    """
    if proxy.transport is None:
        return _init_transport(proxy)
    return not proxy.transport.done


class BufferClient(tf2_ros.BufferInterface):
    """
    tf_service buffer client, can be used like any other TF BufferInterface.
    """

    def __init__(self, server_node_name, keepalive_period=None):
        """
        :param server_node_name: name of the tf_service server ROS node
        :param keepalive_period:
            rospy.Duration defining how often to periodically check the
            connection to the server and try to reconnect in the background
            if it dropped
        """
        tf2_ros.BufferInterface.__init__(self)

        self._mutex = threading.Lock()
        self._reconnection_mutex = threading.Lock()

        with self._reconnection_mutex:
            can_transform_service_full = os.path.join(
                server_node_name, CAN_TRANSFORM_SERVICE_NAME
            )
            self._can_transform_client = rospy.ServiceProxy(
                can_transform_service_full, CanTransform, persistent=True
            )
            _init_transport(self._can_transform_client)

            lookup_transform_service_full = os.path.join(
                server_node_name, LOOKUP_TRANSFORM_SERVICE_NAME
            )
            self._lookup_transform_client = rospy.ServiceProxy(
                lookup_transform_service_full, LookupTransform, persistent=True
            )
            _init_transport(self._lookup_transform_client)

        if keepalive_period is not None:
            self._keepalive_timer = rospy.Timer(
                keepalive_period, self._keepalive_callback, reset=True
            )

    def wait_for_server(self, timeout: rospy.Duration) -> bool:
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
        return _is_valid(self._can_transform_client) and _is_valid(
            self._lookup_transform_client
        )

    def reconnect(self, timeout=rospy.Duration(10)) -> bool:
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
            except rospy.ROSException:
                rospy.logerr("Failed to connect to tf_service server.")
                return False

            self._can_transform_client = rospy.ServiceProxy(
                self._can_transform_client.resolved_name, CanTransform, persistent=True
            )
            _init_transport(self._can_transform_client)
            self._lookup_transform_client = rospy.ServiceProxy(
                self._lookup_transform_client.resolved_name,
                LookupTransform,
                persistent=True,
            )
            _init_transport(self._lookup_transform_client)
        rospy.loginfo(
            "Connected to services %s & %s",
            self._can_transform_client.resolved_name,
            self._lookup_transform_client.resolved_name,
        )
        return True

    async def async_reconnect(self, timeout=None):
        if self._reconnection_mutex.locked():
            rospy.logdebug("Already asynchronously reconnecting to server.")
            return
        rospy.loginfo("Asynchronously trying to reconnect to tf_service server.")
        self.reconnect(timeout)

    def _keepalive_callback(self, _):
        if not self.is_connected():
            asyncio.run(self.async_reconnect())

    def lookup_transform(
        self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)
    ):
        """
        Get the transform from the source frame to the target frame.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        request = LookupTransformRequest()
        request.target_frame = target_frame
        request.source_frame = source_frame
        request.time = time
        request.timeout = timeout
        request.advanced = False

        with self._mutex:
            try:
                response: LookupTransformResponse = self._lookup_transform_client.call(
                    request
                )
                _throw_on_error(response.status)
                return response.transform
            except (rospy.ROSException, rospy.ServiceException):
                raise tf2_ros.TransformException("service call to buffer server failed")

    def lookup_transform_full(
        self,
        target_frame,
        target_time,
        source_frame,
        source_time,
        fixed_frame,
        timeout=rospy.Duration(0.0),
    ):
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
        request = LookupTransformRequest()
        request.target_frame = target_frame
        request.target_time = target_time
        request.source_frame = source_frame
        request.source_time = source_time
        request.fixed_frame = fixed_frame
        request.timeout = timeout
        request.advanced = True

        with self._mutex:
            try:
                response: LookupTransformResponse = self._lookup_transform_client.call(
                    request
                )
                _throw_on_error(response.status)
                return response.transform
            except (rospy.ROSException, rospy.ServiceException):
                raise tf2_ros.TransformException("service call to buffer server failed")

    def can_transform(
        self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)
    ):
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
        request = CanTransformRequest()
        request.target_frame = target_frame
        request.source_frame = source_frame
        request.time = time
        request.timeout = timeout
        request.advanced = False

        with self._mutex:
            try:
                response: CanTransformResponse = self._can_transform_client.call(
                    request
                )
                return response.can_transform
            except (rospy.ROSException, rospy.ServiceException):
                raise tf2_ros.TransformException("service call to buffer server failed")

    def can_transform_full(
        self,
        target_frame,
        target_time,
        source_frame,
        source_time,
        fixed_frame,
        timeout=rospy.Duration(0.0),
    ):
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
        request = CanTransformRequest()
        request.target_frame = target_frame
        request.target_time = target_time
        request.source_frame = source_frame
        request.source_time = source_time
        request.fixed_frame = fixed_frame
        request.timeout = timeout
        request.advanced = True

        with self._mutex:
            try:
                response: CanTransformResponse = self._can_transform_client.call(
                    request
                )
                return response.can_transform
            except (rospy.ROSException, rospy.ServiceException):
                raise tf2_ros.TransformException("service call to buffer server failed")
