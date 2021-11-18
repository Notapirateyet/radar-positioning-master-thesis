#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""This is intended to gather all the published pointclouds from one topic, transform them to the "odom"-frame
and republish them.
It will remove all points every X seconds. (default: Inf)
It can either assume a stationary observer who is only rotating and sending Imu messages on "imu/data", or
a moving observer who sends a pose ("Odometry") on "odometry/filtered"
"""

from typing import List, Tuple

import rospy
from rospy.topics import Publisher
from sensor_msgs.msg import PointCloud, ChannelFloat32, Imu, PointCloud2, PointField
from geometry_msgs.msg import Point32, Point, TransformStamped
from tf2_geometry_msgs import PointStamped
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import *
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
# logging
import csv


def minusP32(point1, point2):
    return Point32(point1.x - point2.x, point1.y - point2.y, point1.z - point1.z)


def addPoints(p1, p2):
    return Point(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z)


def reject_outliers_median(data, m=3.0):
    """Returns a list of bools indicating if they're inliers (true) or outliers (false)"""
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d / (mdev if mdev else 1.0)
    return s < m


def reject_outliers_thresholding(data, threshold=0.0):
    """Returns a list of bools indicating if they're inliers (true) or outliers (false)"""
    return data > threshold


def remove_outliers(points: List[Point], s_chnl: ChannelFloat32, rcs_chnl: ChannelFloat32) -> Tuple[List[Point], ChannelFloat32, ChannelFloat32]:
    """ Detects outliers using either speed or rcs data, and returns the clean data. """

    # Get the values out of the channels and into a numpy array
    extracted_rcs = rcs_chnl.values
    extracted_rcs = np.array(extracted_rcs)
    extracted_speed = s_chnl.values
    extracted_speed = np.array(extracted_speed)
    tmp_points = []
    for cur_p in points:
        tmp_points.append([cur_p.x, cur_p.y, cur_p.z])
    extracted_points = np.array(tmp_points)

    # Create the rejection mask
    # 1 is determined from a high RCS value, 2 is trying to find things moving at a different speed (pedestrians)
    mask1 = reject_outliers_thresholding(extracted_rcs, threshold=0.0)
    mask2 = reject_outliers_median(extracted_speed)
    mask = mask1 & mask2

    # Actually use the mask to remove the points
    masked_rcs = extracted_rcs[mask]
    masked_speed = extracted_speed[mask]
    mask = mask.squeeze()  # I hate this solution -A
    masked_points = extracted_points[mask]

    # Convert back into the input formats
    masked_rcs = masked_rcs.tolist()
    masked_speed = masked_speed.tolist()

    output_points = []
    for cp in masked_points:
        cp = cp.squeeze()  # :/
        tmp_point = Point(x=cp[0], y=cp[1], z=cp[2])
        output_points.append(tmp_point)

    rcs_chnl.values = masked_rcs
    s_chnl.values = masked_speed

    return output_points, s_chnl, rcs_chnl


class CloudGatherer:
    """Saves the total cloud"""

    cloud = PointCloud()
    odom = Odometry()
    last_Imu = Imu()
    position = Point(0, 0, 0)
    keep_old_points = False
    recieved_odometry = False
    last_reset = rospy.Time.from_sec(0)
    KEEP_TIME = np.inf  # secs
    publish_tf = False
    remove_outliers = False
    # Frames i keep messing up
    transform_points_to_frame = "odom"

    def __init__(self):
        self.cloud.header.frame_id = "odom"
        # print(self.cloud.points)
        # print(self.last_Imu.header.stamp) #debug

    def delete_points(self):
        self.cloud = PointCloud()
        self.cloud.header.frame_id = "odom"

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

    def add_RCS_to_cloud_no_rotation(self, channel):
        if self.keep_old_points:
            for cc in channel.values:
                self.cloud.channels[1].values.append(cc)
        else:
            self.cloud.channels[1] = channel

    def rotate_and_add(self, points_base, channels=None, origin_frame="base_link"):
        # Channel must be a ChannelFloat32, describing the speed_radial
        # Check to see if we can do the transform
        if self.last_Imu.header.stamp.secs == 0:
            rospy.logwarn("No imu data recieved yet")
            return  # Next part breaks without IMU data to fall back on
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
        # The math below is from: https://se.mathworks.com/help/robotics/ref/quaternion.rotatepoint.html#d123e37541
        # Rotate and add the points to the internal pointcloud
        rotated_points = []
        for point in points_base:
            try:
                # Convert into transformable datatype
                tmp_point = PointStamped(point=point)
                tmp_point.header.frame_id = origin_frame
                # We convert to the frame assigned to the internal cloud, just assume it's true.
                out_point = self.tfbuffer.transform(
                    tmp_point, self.transform_points_to_frame)
                new_point = Point(out_point.point.x,
                                  out_point.point.y, out_point.point.z)
                # debug
                # rospy.loginfo_once("Rotation success")

            except Exception as e:
                rospy.loginfo_throttle(3.0, e)
                # Convert point to list
                msg_as_tf_quat = [point.x, point.y, point.z, 0]
                # Rotate
                temp1 = quaternion_multiply(rot_tf, msg_as_tf_quat)
                temp2 = quaternion_multiply(
                    temp1, quaternion_conjugate(rot_tf))
                # Convert back to R3
                new_point = Point(temp2[0], temp2[1], temp2[2])
                # Consider position
                new_point = addPoints(new_point, body_position)
            rotated_points.append(new_point)
        self.add_points_to_cloud_no_rotation(rotated_points)
        # Again, why do I name things point Point and points -A

        # Check if we also give it a speed_radial channel
        if channels != None:
            for cur_chnl in channels:
                if self.cloud.channels == []:
                    new_channel = ChannelFloat32()
                    new_channel.name = "Speed_Radial"
                    self.cloud.channels.append(new_channel)
                    new_channel = ChannelFloat32()
                    new_channel.name = "RCS"
                    self.cloud.channels.append(new_channel)
                if cur_chnl.name == "Speed_Radial":
                    self.add_speed_to_cloud_no_rotation(cur_chnl)
                elif cur_chnl.name == "RCS":
                    self.add_RCS_to_cloud_no_rotation(cur_chnl)
                else:
                    rospy.logwarn(
                        "Wrong channel name? detected in Pcloudgatherer008"
                    )
                # If you get a .name on None exception here, the RCS channel is generated wrong

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
                name="Speed_Radial",
                offset=cul_off,
                datatype=PointField.FLOAT32,
                count=1,
            )  # [m/s]
            cul_off = cul_off + 4
            fields.append(f4)
            f5 = PointField(
                name="RCS",
                offset=cul_off,
                datatype=PointField.FLOAT32,
                count=1,
            )
            cul_off = cul_off + 4
            fields.append(f5)

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
                        self.cloud.channels[1].values[indx],
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
        # rospy.loginfo(data)
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

    def set_tfBuffer(self, buffer: tf2_ros.Buffer):
        self.tfbuffer = buffer

    def set_remove_outliers(self, desired_state: bool):
        self.remove_outliers = desired_state


cg = CloudGatherer()  # Global class, hope for the best


def pc_callback(msg, args):
    pub = args
    dur = rospy.Time.now() - cg.last_reset
    if dur.secs > cg.KEEP_TIME:
        cg.delete_points()
        rospy.loginfo("Deleting points")
        cg.last_reset = rospy.Time.now()
    # I don't remeber what creates this error, but if I get it, I just don't add the speeds
    try:
        cg.rotate_and_add(
            msg.points, msg.channels
        )  # takes 3d points and channelfloat32[]
    except IndexError:
        cg.rotate_and_add(msg.points)
    cg.publish(pub)


def xyzspeedrcs_from_pc2(msg: PointCloud2) -> Tuple[List[Point], ChannelFloat32, ChannelFloat32]:
    """Extracts points, radial speeds and radar reliability from a pointcloud2 message, as generated by automotive radars.
    If no rcs is present, it returns a "None" instead."""
    pc2_gen = pc2.read_points(msg, ("x", "y", "z", "Speed_Radial", "RCS"))

    points = []
    spd_chnl = ChannelFloat32()
    spd_chnl.name = "Speed_Radial"
    rcs_chnl = ChannelFloat32()
    rcs_chnl.name = "RCS"

    # Test to check all is present
    try:
        for x, y, z, speed_rad, rcs in pc2_gen:
            points.append(Point(x, y, z))
            spd_chnl.values.append(speed_rad)
            rcs_chnl.values.append(rcs)
    except ValueError:
        rcs_chnl = None
        pc2_gen = pc2.read_points(msg, ("x", "y", "z", "Speed_Radial"))
        for x, y, z, speed_rad, rcs in pc2_gen:
            points.append(Point(x, y, z))
            spd_chnl.values.append(speed_rad)

    return points, spd_chnl, rcs_chnl


def pc2_callback(msg: PointCloud2, args):
    # Extract arguments passed to callback
    pub = args
    # If deletion is chosen
    dur = rospy.Time.now() - cg.last_reset
    if dur.secs > cg.KEEP_TIME:
        cg.delete_points()
        rospy.loginfo("Deleting points")
        cg.last_reset = rospy.Time.now()

    # For the pc2 we need to extract the points
    points, s_chnl, rcs_chnl = xyzspeedrcs_from_pc2(msg)

    # Outlier detection
    points, s_chnl, rcs_chnl = remove_outliers(points, s_chnl, rcs_chnl)

    cg.rotate_and_add(points, [s_chnl, rcs_chnl],
                      origin_frame=msg.header.frame_id)
    cg.publish(pub)


def imu_callback(msg: Imu):
    cg.set_imu(msg)
    if cg.publish_tf == True:
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"  # from this frame
        t.child_frame_id = "odom"  # to this frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        br.sendTransform(t)
        # rospy.loginfo("Sent tf between " + t.header.frame_id +
        #               " and " + t.child_frame_id)


def odometry_callback(msg: Odometry, args: csv.writer):
    logwriter = args
    cg.set_odom(msg)
    cg.publish_tf = False
    if logwriter != None:
        data = [msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z]
        logwriter.writerow(data)


if __name__ == "__main__":
    rospy.init_node("pc_rebublisher", anonymous=False)
    # Set param values. TODO: make them the same format
    cg.keep_old_points = rospy.get_param("~keep_old_points", True)
    cg.set_remove_outliers(rospy.get_param("~remove_outliers", False))
    # Create publishers
    pub = rospy.Publisher("radar/collection", PointCloud2, queue_size=2)
    # Create subscribers and associated things
    log_to_csv = False
    if log_to_csv:
        logfile = open("/home/rincewind/Documents/logs/pcloudgatherer_out003.csv",
                       'w', encoding='UTF8')
        writer = csv.writer(logfile)
        header = ["x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz"]
        writer.writerow(header)
    else:
        writer = None
    i_sub = rospy.Subscriber("imu/data", Imu, imu_callback)
    o_sub = rospy.Subscriber(
        "odometry/filtered", Odometry, odometry_callback, callback_args=writer)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    cg.set_tfBuffer(tfBuffer)

    rospy.Subscriber(
        "radar/target_list_cartesian",
        PointCloud2,
        callback=pc2_callback,
        callback_args=(pub),
        queue_size=10,
    )
    rospy.Subscriber(
        "radar/pointcloud",
        PointCloud,
        callback=pc_callback,
        callback_args=(pub),
        queue_size=2,
    )
    rospy.loginfo("Ready to publish collected pointclouds")
    rospy.spin()
    # end?
    logfile.close()
