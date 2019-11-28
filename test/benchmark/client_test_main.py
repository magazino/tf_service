#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
