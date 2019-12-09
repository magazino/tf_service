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

import argparse
import sys

import rospy
import tf2_ros

import tf_service

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--lookup_frequency", type=float, default=10)
    parser.add_argument("--use_old_version", action="store_true")
    args, _ = parser.parse_known_args(rospy.myargv(sys.argv)[1:])

    rospy.init_node("client_test_py")

    if args.use_old_version:
        buffer = tf2_ros.BufferClient("/tf2_buffer_server")
    else:
        buffer = tf_service.BufferClient("/tf_service")
    buffer.wait_for_server()

    rate = rospy.Rate(args.lookup_frequency)
    while not rospy.is_shutdown():
        try:
            buffer.lookup_transform("map", "odom", rospy.Time(0),
                                    rospy.Duration(1))
        except tf2_ros.TransformException as e:
            rospy.logerr("%s: %s" % (str(type(e)), str(e)))
            break
        rate.sleep()
