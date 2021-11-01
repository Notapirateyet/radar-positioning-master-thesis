#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""This is intended to gather all the published pointclouds from one topic, transform them to tehe "map"-frame 
and republish them.
It can either assume a stationary observer who is only rotating and sending Imu messages on "imu/data", or
a moving observer who sends a pose ("Odometry") on "odometry/filtered"
"""

from typing import List, Tuple
import rospy
from rospy.topics import Publisher
from sensor_msgs.msg import PointCloud, ChannelFloat32, Imu, PointCloud2, PointField
from geometry_msgs.msg import Point32, Point, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import *
import sensor_msgs.point_cloud2 as pc2
import tf2_ros


def minusP32(point1, point2):
    return Point32(point1.x - point2.x, point1.y - point2.y, point1.z - point1.z)


def addPoints(p1, p2):
    return Point(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z)


class CloudGatherer:
    """Saves the total cloud"""

    cloud = PointCloud()
    odom = Odometry()
    last_Imu = Imu()
    position = Point(0, 0, 0)
    keep_old_points = False
    recieved_odometry = False

    def __init__(self) -> None:
        self.cloud.header.frame_id = "map"
        # print(self.cloud.points)
        # print(self.last_Imu.header.stamp) #debug

    def add_points_to_cloud_no_rotation(self, new_points):
        if self.keep_old_points:
            for point in new_points:
                self.cloud.points.append(point)
        else:
            self.cloud.points = new_points

    def add_speed_to_cloud_no_rotation(self, channel):
        if self.keep_old_points:
            for cc in channel.values:
                self.cloud.channels[0].values.append(cc)
        else:
            self.cloud.channels[0] = channel

    def rotate_and_add(self, points, channel=None):
        # Channel must be a ChannelFloat32, describing the speed_radial
        # Check to see if we can do the transform
        if self.last_Imu.header.stamp.secs == 0:
            rospy.logwarn("No imu data recieved yet")
            return
        if self.recieved_odometry:
            rot_tf = [
                self.odom.pose.pose.orientation.x,
                self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z,
                self.odom.pose.pose.orientation.w,
            ]
            body_position = Point(
                self.odom.pose.pose.position.x,
                self.odom.pose.pose.position.y,
                self.odom.pose.pose.position.z,
            )
        else:
            # Quaternion package uses lists
            rot_tf = [
                self.last_Imu.orientation.x,
                self.last_Imu.orientation.y,
                self.last_Imu.orientation.z,
                self.last_Imu.orientation.w,
            ]
            body_position = Point(
                self.position.x,
                self.position.y,
                self.position.z,
            )
            # The following is from: https://se.mathworks.com/help/robotics/ref/quaternion.rotatepoint.html#d123e37541
        # Rotate and add the points to the internal pointcloud
        rotated_points = []
        for point in points:
            # Convert point to list
            msg_as_tf_quat = [point.x, point.y, point.z, 0]
            # Rotate
            temp1 = quaternion_multiply(rot_tf, msg_as_tf_quat)
            temp2 = quaternion_multiply(temp1, quaternion_conjugate(rot_tf))
            # Convert back to R3
            new_point = Point(temp2[0], temp2[1], temp2[2])
            # Consider position
            new_point = addPoints(new_point, body_position)
            rotated_points.append(new_point)
        self.add_points_to_cloud_no_rotation(rotated_points)
        # Again, why do I name things point Point and points -A

        # Check if we also give it a speed_radial channel
        if channel != None:
            if self.cloud.channels == []:
                new_channel = ChannelFloat32()
                new_channel.name = "Speed_Radial"
                self.cloud.channels.append(new_channel)
            if channel.name != "speed_radial":
                rospy.logwarn(
                    "You are converting: "
                    + channel.name
                    + ", to speed_radial. In name only. No value conversion."
                )
            self.add_speed_to_cloud_no_rotation(channel)

    def new_cloud(self, points):
        self.cloud.points = points

    def publish(self, pub: Publisher):
        self.cloud.header.stamp = rospy.Time.now()

        # Create description of data fields
        # Offset = last_offset + bits_in_datatype / 8
        fields = []
        cul_off = 0
        f1 = PointField(name="x", offset=cul_off,
                        datatype=PointField.FLOAT32, count=1)
        fields.append(f1)
        cul_off = cul_off + 4

        f2 = PointField(name="y", offset=cul_off,
                        datatype=PointField.FLOAT32, count=1)
        fields.append(f2)
        cul_off = cul_off + 4

        f3 = PointField(name="z", offset=cul_off,
                        datatype=PointField.FLOAT32, count=1)
        fields.append(f3)
        cul_off = cul_off + 4

        if self.cloud.channels != []:
            f4 = PointField(
                name="speed_radial",
                offset=cul_off,
                datatype=PointField.FLOAT32,
                count=1,
            )  # [m/s]
            cul_off = cul_off + 4
            fields.append(f4)

        # Force the data to be the specified datatype (here, float32)
        # data = np.array([[0, 0, 0, 0], [1, 0, 0, 0.1]], dtype=np.float32)
        data = []
        if self.cloud.channels != []:
            for indx in range(len(self.cloud.points)):
                data.append(
                    [
                        self.cloud.points[indx].x,
                        self.cloud.points[indx].y,
                        self.cloud.points[indx].z,
                        self.cloud.channels[0].values[indx],
                    ]
                )
        else:
            for indx in range(len(self.cloud.points)):
                data.append(
                    [
                        self.cloud.points[indx].x,
                        self.cloud.points[indx].y,
                        self.cloud.points[indx].z,
                    ]
                )
        data = np.array(data, dtype=np.float32)

        new_cloud = pc2.create_cloud(self.cloud.header, fields, data)
        # print(self.cloud.points)
        # pub.publish(self.cloud)
        pub.publish(new_cloud)

    def get_cloud(self) -> PointCloud:
        return self.cloud

    def set_imu(self, new_imu):
        self.last_Imu = new_imu

    def get_imu(self) -> Imu:
        return self.last_Imu

    def set_odom(self, msg: Odometry):
        self.odom = msg
        self.recieved_odometry = True

    def get_odom(self) -> Odometry:
        return self.odom

    def set_position(self, new_pos):
        self.position = new_pos


publish_tf = False
cg = CloudGatherer()  # Global class, hope for the best


def pc_callback(msg, args):
    pub = args
    try:
        cg.rotate_and_add(
            msg.points, msg.channels[0]
        )  # takes 3d points and channelfloat32
    except IndexError:
        cg.rotate_and_add(msg.points)
    cg.publish(pub)


def xyzspeed_from_pc2(
    pc2: PointCloud2,
) -> Tuple[List[Point], ChannelFloat32]:

    # Assume height is always 1, even though it isn't
    if pc2.height != 1:
        rospy.logwarn("This type of pointcloud is not supported")
        return
    if pc2.is_bigendian:
        rospy.logwarn(
            "Pointcloud is bigendian, this script is probably wrong?")
    # print(msg.data)
    # print(type(msg.data))
    # print(type(msg.data[0]))
    data = pc2.data
    s_chnl = ChannelFloat32()
    s_chnl.name = "speed_radial"
    points = []

    for w in range(pc2.width):
        # Need to combine 4 uint8 into a float32
        indx = w * pc2.point_step
        x = np.array(
            [data[indx], data[indx + 1], data[indx + 2], data[indx + 3]], np.uint8
        )
        y = np.array(
            [data[indx + 4], data[indx + 5], data[indx + 6], data[indx + 7]], np.uint8
        )
        z = np.array(
            [data[indx + 8], data[indx + 9],
                data[indx + 10], data[indx + 11]], np.uint8
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
    return points, s_chnl


def pc2_callback(msg: PointCloud2, args):
    pub = args
    # For the pc2 we need to extract the points
    points, s_chnl = xyzspeed_from_pc2(msg)

    cg.rotate_and_add(points, s_chnl)
    # debug
    # cg.add_to_cloud_no_rotation(points)
    cg.publish(pub)


def imu_callback(msg: Imu):
    cg.set_imu(msg)
    if publish_tf == True:
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "imu_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        br.sendTransform(t)


def odometry_callback(msg):
    cg.set_odom(msg)


if __name__ == "__main__":
    rospy.init_node("pc_rebublisher", anonymous=False)
    cg.keep_old_points = rospy.get_param("~keep_old_points", False)
    pub = rospy.Publisher("radar/collection", PointCloud2, queue_size=2)

    i_sub = rospy.Subscriber("imu/data", Imu, imu_callback)
    o_sub = rospy.Subscriber("odometry/filtered", Odometry, odometry_callback)

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
    rospy.loginfo(
        "Ready to publish pointclouds in odom frame disguised in imu_link")
    rospy.spin()
