#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import unittest

import rosnode
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

import tf_service

# See rostest launch file.
EXPECTED_SERVER_NAME = "/tf_service"
EXPECTED_TARGET_FRAME = "map"
EXPECTED_SOURCE_FRAME = "odom"


class ClientRostest(unittest.TestCase):
    def test_wait_for_server_succeeds(self):
        buffer = tf_service.BufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        self.assertTrue(buffer.is_connected())

    def test_wait_for_server_fails(self):
        buffer = tf_service.BufferClient("/wrong_server_name")
        self.assertFalse(buffer.wait_for_server(rospy.Duration(0.1)))
        self.assertFalse(buffer.is_connected())

    def test_can_transform(self):
        buffer = tf_service.BufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        self.assertTrue(
            buffer.can_transform(EXPECTED_TARGET_FRAME, EXPECTED_SOURCE_FRAME,
                                 rospy.Time(0), rospy.Duration(0.1)))
        self.assertFalse(
            buffer.can_transform("bla", "blub", rospy.Time(0),
                                 rospy.Duration(0.1)))

    def test_can_transform_advanced(self):
        buffer = tf_service.BufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        self.assertTrue(
            buffer.can_transform_full(EXPECTED_TARGET_FRAME,
                                      rospy.Time(0), EXPECTED_SOURCE_FRAME,
                                      rospy.Time(0), EXPECTED_TARGET_FRAME,
                                      rospy.Duration(0.1)))
        self.assertFalse(
            buffer.can_transform_full("bla", rospy.Time(0), "blub",
                                      rospy.Time(0), "bla",
                                      rospy.Duration(0.1)))

    def test_lookup_transform(self):
        buffer = tf_service.BufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        buffer.lookup_transform(EXPECTED_TARGET_FRAME, EXPECTED_SOURCE_FRAME,
                                rospy.Time(0), rospy.Duration(0.1))
        with self.assertRaises(tf2_ros.LookupException):
            buffer.lookup_transform("bla", "blub", rospy.Time(0),
                                    rospy.Duration(0.1))

    def test_lookup_transform_advanced(self):
        buffer = tf_service.BufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        buffer.lookup_transform_full(EXPECTED_TARGET_FRAME,
                                     rospy.Time(0), EXPECTED_SOURCE_FRAME,
                                     rospy.Time(0), EXPECTED_TARGET_FRAME,
                                     rospy.Duration(0.1))
        with self.assertRaises(tf2_ros.LookupException):
            buffer.lookup_transform_full("bla", rospy.Time(0), "blub",
                                         rospy.Time(0), "bla",
                                         rospy.Duration(0.1))

    def test_transform(self):
        """
        Smoke test for checking that the implicitly inherited transform() method
        from BufferInterface is usable.
        """
        buffer = tf_service.BufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = EXPECTED_SOURCE_FRAME
        buffer.transform(pose, EXPECTED_TARGET_FRAME)

    def test_transform_full(self):
        """
        Smoke test for checking that the implicitly inherited transform_full()
        method from BufferInterface is usable.
        """
        buffer = tf_service.BufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        pose = PoseStamped()
        now = rospy.Time.now()
        pose.header.stamp = now
        pose.header.frame_id = EXPECTED_SOURCE_FRAME
        buffer.transform_full(
            pose, EXPECTED_TARGET_FRAME, now, EXPECTED_SOURCE_FRAME, rospy.Duration(1)
        )

    def test_keepalive(self):
        buffer = tf_service.BufferClient(EXPECTED_SERVER_NAME,
                                         keepalive_period=rospy.Duration(1))
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        # Assuming the server respawns after ~5 seconds.
        rosnode.kill_nodes(node_names=[EXPECTED_SERVER_NAME])
        rospy.sleep(1)
        self.assertFalse(buffer.is_connected())
        rospy.sleep(10)
        self.assertTrue(buffer.is_connected())


if __name__ == "__main__":
    import rostest
    rospy.init_node("client_rostest_py")
    rostest.rosrun("tf_service", "client_rostest_py", ClientRostest)
