# Copyright 2025 Magazino GmbH
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

import typing

import rospy
import rospy.core
from rospy.exceptions import TransportInitError
from rospy.impl.tcpros_base import TCPROSTransport

PING_TIMEOUT = 0.1


class PersistentService(rospy.ServiceProxy):
    """
    Persistent service proxy with extensions that are missing from rospy.
    """

    def __init__(self, name: str, service_class: typing.Any) -> None:
        # Re-declaration with typing to make mypy happy:
        self.transport: typing.Optional[TCPROSTransport] = None

        super().__init__(name, service_class, persistent=True)
        self.init_transport()

    def is_valid(self) -> bool:
        """
        rospy doesn't have an equivalent of ros::ServiceClient::isValid().
        This should do the trick instead...
        """
        if self.transport is None:
            return False

        # With a persistent service, the self.transport is kept after service calls.
        # Transport loss is only noticed when something is transmitted.
        # "Ping" the remote address with a header handshake to check if it's alive.
        dest_addr, dest_port = self.transport.dest_address
        try:
            self.transport.connect(
                dest_addr, dest_port, self.transport.endpoint_id, timeout=PING_TIMEOUT
            )
        except TransportInitError as e:
            rospy.logerr("Lost connection to %s", self.resolved_name)
            self.transport = None
            return False

        return True

    def init_transport(self) -> bool:
        """
        Transport initialization of persistent rospy service is deferred to
        first time call() is called.
        This function allows to establish the transport independently of call().
        """
        if self.transport is not None:
            return True

        try:
            service_uri = self._get_service_uri(self.request_class())
            dest_addr, dest_port = rospy.core.parse_rosrpc_uri(service_uri)
        except rospy.ServiceException as error:
            rospy.logerr(str(error))
            return False

        try:
            transport = TCPROSTransport(self.protocol, self.resolved_name)
            transport.buff_size = self.buff_size
            transport.connect(dest_addr, dest_port, service_uri)
            if self.persistent:
                self.transport = transport
        except TransportInitError as error:
            rospy.logerr(str(error))
            return False

        return True
