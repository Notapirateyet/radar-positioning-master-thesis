#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""This is intended to gather all the published pointclouds from one topic, transform them to /odom and republish them"""

import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32, Point
import tf2_ros
from tf.transformations import *


def minusP32(point1, point2):
    return Point32(point1.x - point2.x, point1.y - point2.y, point1.z - point1.z)


def pc_callback(msg, args):
    pub = args[0]
    tfBuffer = args[1]
    try:
        msg = tfBuffer.transform(msg, "odom")
        print("found")

    except:
        # print(rospy.Time.now())
        trans = tfBuffer.lookup_transform("imu_link", "odom", rospy.Time(0))

        rot = trans.transform.rotation  # Should give a quat
        rot_tf = [rot.x, rot.y, rot.z, rot.w]  # For the tf package
        # The following is from https://forum.unity.com/threads/rotate-a-point-around-a-second-point-using-a-quaternion.504996/
        # Might not be true. Check matlab, they had an algorithm
        # It should rotate "point" around "center" using the rotation "rot"
        # Now from: https://se.mathworks.com/help/robotics/ref/quaternion.rotatepoint.html#d123e37541
        center = Point32(
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        )
        new_cloud = []
        for point in msg.points:
            msg_as_tf_quat = [point.x, point.y, point.z, 0]  # Convert point to tf quat
            # Rotate
            temp1 = quaternion_multiply(rot_tf, msg_as_tf_quat)
            temp2 = quaternion_multiply(temp1, quaternion_conjugate(rot_tf))
            # Convert back to R3
            new_point = Point32(temp2[0], temp2[1], temp2[2])

            # temp_point = point - center  # Center
            # new_point = rot * temp_point  # Rotate
            # new_point = new_point + center  # Bring back
            new_cloud.append(new_point)
        msg.points = new_cloud

    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "imu_link"
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pc_rebublisher")
    pub = rospy.Publisher("radar/collection", PointCloud, queue_size=4)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.Subscriber(
        "radar/pointcloud",
        PointCloud,
        callback=pc_callback,
        callback_args=(pub, tfBuffer),
        queue_size=4,
    )
    print("Ready to publish pointclouds in /odom")
    rospy.spin()
