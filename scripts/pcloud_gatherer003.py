#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""This is intended to gather all the published pointclouds from one topic, transform them to "odom" and republish them.
It currently assumes the observer is only rotating, and not moving. Should be extended to recieve odometry messages and take the orientation from there.
"""

import rospy
from rospy.topics import Publisher
from sensor_msgs.msg import PointCloud, ChannelFloat32, Imu, PointCloud2
from geometry_msgs.msg import Point32, Point
import numpy as np
from tf.transformations import *
import sensor_msgs.point_cloud2 as pc2


def minusP32(point1, point2):
    return Point32(point1.x - point2.x, point1.y - point2.y, point1.z - point1.z)


def addPoints(p1, p2):
    return Point(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z)


class CloudGatherer:
    """Saves the total cloud"""

    cloud = PointCloud()
    last_Imu = Imu()
    position = Point(0, 0, 0)
    keep_old_points = False

    def __init__(self) -> None:
        self.cloud.header.frame_id = "map"
        channel = ChannelFloat32()
        channel.name = "Speed_Radial"
        self.cloud.channels.append(channel)
        # print(self.cloud.points)
        # print(self.last_Imu.header.stamp) #debug

    def add_to_cloud_no_rotation(self, new_points):
        if self.keep_old_points:
            for point in new_points:
                self.cloud.points.append(point)
        else:
            self.cloud.points = new_points

    def rotate_and_add(self, points, channel=None):
        # Channel must be a ChannelFloat32, describing the speed
        # Check to see if we can do the transform
        if self.last_Imu.header.stamp.secs == 0:
            rospy.logwarn("No imu data recieved yet")
            return
        # Quaternion package uses lists
        rot_tf = [
            self.last_Imu.orientation.x,
            self.last_Imu.orientation.y,
            self.last_Imu.orientation.z,
            self.last_Imu.orientation.w,
        ]
        # The following is from: https://se.mathworks.com/help/robotics/ref/quaternion.rotatepoint.html#d123e37541
        # "center" is usused, hmm
        body_position = Point(
            self.position.x,
            self.position.y,
            self.position.z,
        )
        rotated_points = []
        for point in points:
            msg_as_tf_quat = [point.x, point.y, point.z, 0]  # Convert point to list
            # Rotate
            temp1 = quaternion_multiply(rot_tf, msg_as_tf_quat)
            temp2 = quaternion_multiply(temp1, quaternion_conjugate(rot_tf))
            # Convert back to R3
            new_point = Point(temp2[0], temp2[1], temp2[2])
            new_point = addPoints(new_point, body_position)
            rotated_points.append(new_point)
        self.add_to_cloud_no_rotation(rotated_points)
        if channel != None:
            for cc in channel.values:
                self.cloud.channels[0].values.append(cc)

    def new_cloud(self, points):
        self.cloud.points = points

    def publish(self, pub: Publisher):
        self.cloud.header.stamp = rospy.Time.now()
        t_points = []
        for p in self.cloud.points:
            t_points.append([p.x, p.y, p.z])
        new_cloud = pc2.create_cloud_xyz32(self.cloud.header, t_points)
        # print(self.cloud.points)
        # pub.publish(self.cloud)
        pub.publish(new_cloud)

    def get_cloud(self) -> PointCloud:
        return self.cloud

    def set_imu(self, new_imu):
        self.last_Imu = new_imu

    def get_imu(self) -> Imu:
        return self.last_Imu

    def set_position(self, new_pos):
        self.position = new_pos


cg = CloudGatherer()  # Global class, hope for the best


def pc_callback(msg, args):
    pub = args
    cg.rotate_and_add(msg.points, msg.channels[0])
    cg.publish(pub)


def pc2_callback(msg: PointCloud2, args):
    pub = args
    # For the pc2 we need to extract the points
    # Assume it is always this, even though it isn't
    if msg.height != 1:
        rospy.logwarn("This type of pointcloud is not supported")
        return
    if msg.is_bigendian:
        rospy.logwarn("Pointcloud is bigendian, this script is probably wrong?")
    data_things = len(msg.fields)
    # print(msg.data)
    # print(type(msg.data))
    # print(type(msg.data[0]))
    data = msg.data
    s_chnl = ChannelFloat32()
    s_chnl.name = "Speed_Radial"
    points = []
    for w in range(msg.width):
        # Need to combine 4 uint8 into a float32
        indx = w * msg.point_step
        x = np.array(
            [data[indx], data[indx + 1], data[indx + 2], data[indx + 3]], np.uint8
        )
        y = np.array(
            [data[indx + 4], data[indx + 5], data[indx + 6], data[indx + 7]], np.uint8
        )
        z = np.array(
            [data[indx + 8], data[indx + 9], data[indx + 10], data[indx + 11]], np.uint8
        )
        speed = np.array(
            [data[indx + 12], data[indx + 13], data[indx + 14], data[indx + 15]],
            np.uint8,
        )

        x = x.view(np.float32)
        y = y.view(np.float32)
        z = z.view(np.float32)
        speed = speed.view(np.float32)
        # Assume the first three are x y z

        point = Point(x, y, z)
        points.append(
            point
        )  # points point and Point are different things -> consider renaming
        s_chnl.values.append(speed)

    cg.rotate_and_add(points, s_chnl)
    # debug
    # cg.add_to_cloud_no_rotation(points)
    cg.publish(pub)


def imu_callback(msg):
    cg.set_imu(msg)


if __name__ == "__main__":
    rospy.init_node("pc_rebublisher", anonymous=False)
    pub = rospy.Publisher("radar/collection", PointCloud2, queue_size=2)

    sub = rospy.Subscriber("imu/data", Imu, imu_callback)

    rospy.Subscriber(
        "radar1/target_list_cartesian",
        PointCloud2,
        callback=pc2_callback,
        callback_args=(pub),
        queue_size=2,
    )
    rospy.Subscriber(
        "radar/pointcloud",
        PointCloud,
        callback=pc_callback,
        callback_args=(pub),
        queue_size=2,
    )
    rospy.loginfo("Ready to publish pointclouds in odom frame disguised in imu_link")
    rospy.spin()
