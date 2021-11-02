#!/usr/bin/env python
# -*- coding: utf_8 -*-

"""This is supposed to take radar pointclouds, and then publish an odometry estimate with only velocity"""

from typing import Tuple, List
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, ChannelFloat32, PointCloud
from geometry_msgs.msg import Point
from tf.transformations import unit_vector
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from sklearn import linear_model

# Distance from median, or edge case, distance from mean
# Outlier rejection by Benjamin Bannier on stackoverflow


def reject_outliers(data, m=3.0):
    """Returns a list of bools indicating if they're inliers (true) or outliers (false)"""
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d / (mdev if mdev else 1.0)
    return s < m


def reject_outliers2(data):
    """Returns a list of bools indicating if they're inliers (true) or outliers (false).
    Uses RANSAC for this."""
    if len(data) < 3:
        return reject_outliers(data)
    ransac = linear_model.RANSACRegressor()
    med = np.median(data)
    y = np.ones_like(data) * med

    ransac.fit(data, y)
    inlier_mask = ransac.inlier_mask_
    return inlier_mask


def xyzspeed_from_pc2v2(
    pc2: PointCloud2,
) -> Tuple[List[Point], np.array]:

    # Assume height is always 1, even though it isn't
    if pc2.height != 1:
        rospy.logwarn("This type of pointcloud is not supported")
        return
    if pc2.is_bigendian:
        rospy.logwarn("Pointcloud is bigendian, this script is probably wrong?")
    # print(msg.data)
    # print(type(msg.data))
    # print(type(msg.data[0]))
    data = pc2.data
    points = []
    speeds = []

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
        speeds.append(speed)
    speeds = np.array(speeds)
    return points, speeds


def estimate_ownship_velocity(
    points: List[Point], speeds: np.array
) -> Tuple[np.array, np.array]:
    """Calculates the ownship velocity based on radar dobbler readings.
    Return the ownshib velocity as [v_x, v_y, v_z], in the sensor frame"""
    # Maybe if we have a good pointcloud package, we combine points and speeds?
    # Steps:
    # 1. Find the unit vectors of the measurements (x)
    # 2. Do outlier sorting on the speed measurements (x)
    # 3. Estimate velocities (x)
    #       3.1 Estimation algorithm:
    # phi = rs_u; # rs_u -> unit vectors
    # y = vR'; # vR -> Radial velocities
    # theta_prim = (phi'*phi)^(-1)*phi'*y'; # The estimated velocities
    points_np = np.array([[e.x, e.y, e.z] for e in points])
    units = unit_vector(points_np, 1)
    units = units.squeeze()
    # rospy.loginfo(speeds)
    ou_indx = reject_outliers(speeds)
    # rospy.loginfo(ou_indx.T)
    speeds = speeds[ou_indx]
    units = units[np.squeeze(ou_indx.T), :]  # maybe [indx,:]
    # rospy.loginfo(units)

    m2 = np.linalg.pinv(np.matmul(units.T, units))
    m3 = np.matmul(m2, units.T)
    theta_prim = np.matmul(m3, speeds)
    # transform from static thigns speed to our speed # hack
    theta_prim = theta_prim * (-1.0)
    var = np.var(speeds - np.matmul(units, theta_prim))
    velocity_cov = m2 * var

    return theta_prim, velocity_cov


def create_odom_and_pub(velocities, pub, velocity_covariance=-1):
    # Create the odometry message
    odm = Odometry()
    odm.header.stamp = rospy.Time.now()
    odm.header.frame_id = "radar_link"
    odm.child_frame_id = "radar_link"
    # 6x6
    odm.twist.covariance = np.zeros(36)
    if len(velocity_covariance) != 9:
        odm.twist.covariance[0] = 0.2  # x cov
        odm.twist.covariance[7] = 1.0  # y cov
        odm.twist.covariance[14] = 5.0  # z cov
    else:
        temp = odm.twist.covariance
        temp = temp.reshape(6, 6)
        temp[0:3, 0:3] = velocity_covariance.reshape(3, 3)
        odm.twist.covariance = temp.reshape(36)
    odm.twist.twist.linear.x = velocities[0]
    odm.twist.twist.linear.y = velocities[1]
    odm.twist.twist.linear.z = velocities[2]

    # The pose of this message is technically not wrong, since it is in the radar frame, and that rotation/position is always in that frame...
    pub.publish(odm)


def pc2_callback(msg: PointCloud2, pub: rospy.Publisher):
    # This function should recieve a pc2 cloud, get the xyz
    # and speeds and pass them to a secondary, more easy to debug function.
    # let's call it... estimate_ownship_velocity!
    # Maybe then put it in a odometry message and publish it?
    points, velocities = xyzspeed_from_pc2v2(msg)
    ownship_velocity, ownship_velocity_cov = estimate_ownship_velocity(
        points, velocities
    )
    rospy.logdebug(ownship_velocity_cov)
    create_odom_and_pub(ownship_velocity, pub, ownship_velocity_cov)


def pc1_callback(msg: PointCloud, pub: rospy.Publisher):
    points = msg.points
    velocities = msg.channels[0].values
    velocities = np.array([velocities])
    velocities = velocities.T

    ownship_velocity, ownship_velocity_cov = estimate_ownship_velocity(
        points, velocities
    )
    rospy.logdebug(ownship_velocity_cov)
    create_odom_and_pub(ownship_velocity, pub, ownship_velocity_cov)


if __name__ == "__main__":
    rospy.init_node("ego_velocity_estimator", anonymous=False)
    # TODO: Subscribe to a pc2
    pub = rospy.Publisher("radar1/velocity_estimate", Odometry, queue_size=1)

    sub1 = rospy.Subscriber(
        "radar1/target_list_cartesian",
        PointCloud2,
        callback=pc2_callback,
        callback_args=(pub),
        queue_size=1,
    )
    # Debug
    # subtemp = rospy.Subscriber(
    #     "radar/collection",
    #     PointCloud2,
    #     callback=pc2_callback,
    #     callback_args=(pub),
    #     queue_size=1,
    # )
    # This below is debugging from my own measurements
    sub2 = rospy.Subscriber(
        "radar/pointcloud",
        PointCloud,
        callback=pc1_callback,
        callback_args=(pub),
        queue_size=1,
    )
    rospy.loginfo("Ready to estimate ownship velocities")
    rospy.spin()
