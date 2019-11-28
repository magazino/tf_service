#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest

import rospy
import tf2_ros

from tf_service import SimpleBufferClient

# See rostest launch file.
EXPECTED_SERVER_NAME = "/tf_service"
EXPECTED_TARGET_FRAME = "map"
EXPECTED_SOURCE_FRAME = "odom"


class ClientRostest(unittest.TestCase):
    def test_wait_for_server_succeeds(self):
        buffer = SimpleBufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        self.assertTrue(buffer.client.is_connected())

    def test_wait_for_server_fails(self):
        buffer = SimpleBufferClient("/wrong_server_name")
        self.assertFalse(buffer.wait_for_server(rospy.Duration(0.1)))
        self.assertFalse(buffer.client.is_connected())

    def test_can_transform(self):
        buffer = SimpleBufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        self.assertTrue(
            buffer.can_transform(EXPECTED_TARGET_FRAME, EXPECTED_SOURCE_FRAME,
                                 rospy.Time(0), rospy.Duration(0.1)))
        self.assertFalse(
            buffer.can_transform("bla", "blub", rospy.Time(0),
                                 rospy.Duration(0.1)))

    def test_can_transform_advanced(self):
        buffer = SimpleBufferClient(EXPECTED_SERVER_NAME)
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
        buffer = SimpleBufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        buffer.lookup_transform(EXPECTED_TARGET_FRAME, EXPECTED_SOURCE_FRAME,
                                rospy.Time(0), rospy.Duration(0.1))
        with self.assertRaises(tf2_ros.LookupException):
            buffer.lookup_transform("bla", "blub", rospy.Time(0),
                                    rospy.Duration(0.1))

    def test_lookup_transform_advanced(self):
        buffer = SimpleBufferClient(EXPECTED_SERVER_NAME)
        self.assertTrue(buffer.wait_for_server(rospy.Duration(0.1)))
        buffer.lookup_transform_full(EXPECTED_TARGET_FRAME,
                                     rospy.Time(0), EXPECTED_SOURCE_FRAME,
                                     rospy.Time(0), EXPECTED_TARGET_FRAME,
                                     rospy.Duration(0.1))
        with self.assertRaises(tf2_ros.LookupException):
            buffer.lookup_transform_full("bla", rospy.Time(0), "blub",
                                         rospy.Time(0), "bla",
                                         rospy.Duration(0.1))


if __name__ == "__main__":
    import rostest
    rostest.rosrun("tf_service", "client_rostest_py", ClientRostest)
