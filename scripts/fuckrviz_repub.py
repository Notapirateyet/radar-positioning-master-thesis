#!/usr/bin/env python
# -*- coding: utf_8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, PointCloud


def pc_callback(msg: PointCloud, pub):
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("fuckrviz_republisher", anonymous=False)

    pub = rospy.Publisher("radar/rebub", PointCloud, queue_size=1)

    sub1 = rospy.Subscriber(
        "radar/pointcloud",
        PointCloud,
        callback=pc_callback,
        callback_args=(pub),
    )
    rospy.loginfo("Started to republish pointclouds with a new timestamp")
    rospy.spin()
