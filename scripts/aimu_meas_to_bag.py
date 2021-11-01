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
#   gps_imu008.txt:
# AccX; AccY; AccZ; GyrX; GyrY; GyrZ; MagX; MagY; MagZ; q0; q1; q2; q3; Lat; Lon; Alt;
# 0.577106; -1.595253; 9.691074; -0.015462; -0.085637; -0.031782; -0.498718; 0.392302; -1.113129; 0.657957; -0.075850; 0.041662; -0.748066; 0.000000; 0.000000; 0.000000;
#   only_imu008.txt:
# Time; AccX; AccY; AccZ; GyrX; GyrY; GyrZ; MagX; MagY; MagZ; q0; q1; q2; q3; Roll; Pitch; Yaw;
# 764569;1.400392; -2.585567; 9.507402;-0.001298; -0.017118; 0.039171;-0.461458; 0.216490; -0.896129;0.283287; 0.008678; -0.147746; 0.947546;-16.049371; -5.748027; 147.520973;
# 764669;1.443853; -2.659463; 9.465131;-0.032438; 0.006163; 0.039612;-0.461486; 0.217926; -0.896093;0.283149; 0.008643; -0.147860; 0.947570;-16.064016; -5.745606; 147.537500;

# PacketCounter;SampleTimeFine;Year;Month;Day;Second;UTC_Nano;UTC_Year;UTC_Month;UTC_Day;UTC_Hour;UTC_Minute;UTC_Second;UTC_Valid;
# Acc_X;Acc_Y;Acc_Z;FreeAcc_E;FreeAcc_N;FreeAcc_U;Gyr_X;Gyr_Y;Gyr_Z;Mag_X;Mag_Y;Mag_Z;VelInc_X;VelInc_Y;VelInc_Z;OriInc_q0;
# OriInc_q1;OriInc_q2;OriInc_q3;Quat_q0;Quat_q1;Quat_q2;Quat_q3;Roll;Pitch;Yaw;Latitude;Longitude;Altitude;Vel_E;Vel_N;Vel_U

# Start time of gps_imu is unknown, but the "pause" is 0.1 s, resulting in 10 Hz, not counting lag.
# It measured for 180 seconds exactly, but only got 1634 data points, giving 9.0777 samples/s (Hz)

# The "time" of only_imu is "fine", but obviously not posix/unix. Documentation says it's arbritary "sample time in 10kHz clock ticks"
# Some investigation suggests it is also 100 Hz, but very much more regular

# They were started within ~4seconds of each other, but, def not at the same time, glhf.

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: csv_to_bag.py csv-file output-bag-file start_time")
    else:
        if len(sys.argv) < 4:
            est_time = 1632412800.0  # Unix time, just, a guess :/
            #          1632478228 # seconds
        else:
            est_time = sys.argv[3]

        df = pd.read_csv(sys.argv[1], sep=";")
        # df = pd.read_csv("~/Meas_2021-09-23/gps_imu008.txt", sep=";")
        # df = pd.read_csv("~/Meas_2021-09-23/only_imu008.txt", sep=";")
        try:
            last_tstmp = df["SampleTimeFine"][0]
            includes_time = True
            that_one_index = "Acc_X"  # Fuck this in particular
        except:
            includes_time = False
            est_sample_time = 180.0 / len(df)  # s/sample
            that_one_index = "AccX"  #

        with rosbag.Bag(sys.argv[2], "w") as bag:
            for row in range(df.shape[0]):
                timestamp = rospy.Time(est_time)
                if includes_time:
                    # print((df["Time"][row] - last_tstmp) / 10000.0)
                    # print(last_tstmp)
                    # print(df["Time"][row])
                    est_time = est_time + (
                        (df["SampleTimeFine"][row] - last_tstmp) / 10000.0
                    )  # seconds, based on sensor internal 10kHz
                    last_tstmp = df["SampleTimeFine"][row]
                else:
                    est_time = est_time + est_sample_time
                timestamp = rospy.Time.from_sec(est_time)

                imu_msg = Imu()
                imu_msg.header.stamp = timestamp
                imu_msg.header.frame_id = "imu_link"
                imu_msg.angular_velocity.x = df["Gyr_X"][row]
                imu_msg.angular_velocity.y = df["Gyr_Y"][row]
                imu_msg.angular_velocity.z = df["Gyr_Z"][row]
                imu_msg.linear_acceleration.x = df[that_one_index][row]
                imu_msg.linear_acceleration.y = df["Acc_Y"][row]
                imu_msg.linear_acceleration.z = df["Acc_Z"][row]

                imu_msg.orientation.x = df["Quat_q1"][row]
                imu_msg.orientation.y = df["Quat_q2"][row]
                imu_msg.orientation.z = df["Quat_q3"][row]
                imu_msg.orientation.w = df["Quat_q0"][row]

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
                if includes_time:
                    bag.write("/imu/data", imu_msg, timestamp)
                else:
                    bag.write("/imu/backup", imu_msg, timestamp)

                if "Lat" in df.columns:
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = timestamp
                    gps_msg.header.frame_id = "imu_link"

                    # Populate the data elements for GPS
                    gps_msg.latitude = df[" Lat"][row]
                    gps_msg.longitude = df[" Lon"][row]
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
        print("Done")
