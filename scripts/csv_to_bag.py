#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This scripts can be used to convert a specific format of csv file containing imu and gps data to a rosbag.
Note that the magnetic field is measured but unused and not reported.
Usage: python csv_to_bag.py input-csv-file output-bag-file
"""

import rospy
import rosbag
import sys
import pandas as pd
import tf
import geometry_msgs
from sensor_msgs.msg import Imu, NavSatFix


# Example data directly from the source
#    15 1631882320336             0  55.783333  12.516032   4.975442  11.168701 -124.600115 -1.954746 0.783796 9.490111 0.013786 -0.019789 0.014284 -0.032720 0.379620 -1.786982 7552.000000
#    16 1631882320438             0  55.783333  12.516033   5.036304  11.140941 -124.575213 -1.947392 0.771770 9.484950 0.009554 -0.004418 0.004683 -0.033277 0.382476 -1.787371 7552.000000
#    17 1631882320539 1631882322387  55.783334  12.516035   4.758693  11.290466 -124.517412 -1.947392 0.771770 9.484950 -0.002124 -0.001032 0.019792 -0.028842 0.388080 -1.795953 7552.000000
#    18 1631882320640 1631882322487  55.783334  12.516036   4.813201  11.297440 -124.412282 -1.920883 0.776817 9.484282 0.005469 0.002823 0.017789 -0.030885 0.389065 -1.794823 7552.000000

# What the coloumns are each representing
# index (-),
# systemtid (ms),
# gpstid (ms),
# lat (deg),
# lon (deg),
# roll (deg),
# pitch (deg),
# yaw (deg),
# ax (-),
# ay (-),
# az (-),
# wx (-),
# wy (-),
# wz (-),
# mx (-),
# my (-),
# mz (-),
# temperatur (-)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: csv_to_bag.py csv-file bag-file")
    else:
        df = pd.read_csv(sys.argv[1], delim_whitespace=True)
        # df = pd.read_csv("imu_1631882318802.log", delim_whitespace=True)
        with rosbag.Bag(sys.argv[2], "w") as bag:
            for row in range(df.shape[0]):
                if df["gpstid"][row] != 0:
                    timestamp = rospy.Time.from_sec(df["gpstid"][row] / 1000.0)
                    imu_msg = Imu()
                    imu_msg.header.stamp = timestamp
                    imu_msg.header.frame_id = "imu_link"
                    imu_msg.angular_velocity.x = df["wx"][row]
                    imu_msg.angular_velocity.y = df["wy"][row]
                    imu_msg.angular_velocity.z = df["wz"][row]
                    imu_msg.linear_acceleration.x = df["ax"][row]
                    imu_msg.linear_acceleration.y = df["ay"][row]
                    imu_msg.linear_acceleration.z = df["az"][row]
                    orient = tf.transformations.quaternion_from_euler(
                        df["roll"][row], df["pitch"][row], df["yaw"][row]
                    )  # Rotation
                    imu_msg.orientation.x = orient[0]
                    imu_msg.orientation.y = orient[1]
                    imu_msg.orientation.z = orient[2]
                    imu_msg.orientation.w = orient[3]
                    cov = 1.0
                    imu_msg.orientation_covariance = [
                        cov,
                        0,
                        0,
                        0,
                        cov,
                        0,
                        0,
                        0,
                        cov,
                    ]
                    imu_msg.angular_velocity_covariance = [
                        cov,
                        0,
                        0,
                        0,
                        cov,
                        0,
                        0,
                        0,
                        cov,
                    ]
                    imu_msg.linear_acceleration_covariance = [
                        cov,
                        0,
                        0,
                        0,
                        cov,
                        0,
                        0,
                        0,
                        cov,
                    ]
                    # print(imu_msg)
                    # print(orient)
                    bag.write("/imu/data", imu_msg, timestamp)

                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = timestamp
                    gps_msg.header.frame_id = "imu_link"

                    # Populate the data elements for GPS
                    gps_msg.latitude = df["lat"][row]
                    gps_msg.longitude = df["lon"][row]
                    gps_msg.altitude = float("NaN")
                    cov = 50.0
                    gps_msg.position_covariance = [
                        cov,
                        0,
                        0,
                        0,
                        cov,
                        0,
                        0,
                        0,
                        cov,
                    ]
                    gps_msg.position_covariance_type = (
                        gps_msg.COVARIANCE_TYPE_APPROXIMATED
                    )

                    bag.write("/gps/raw", gps_msg, timestamp)
