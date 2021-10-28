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
import math
import tf
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

# Remember that pyhton is 0-indexed
# 1. Time [ms], 2. HostTimestamp, 3. Sensorno., 4. Cycle_Duration [s], 5. Nof_Tgts, 6. Cycle_Count, 7. Object_Number,
# 8. Range [m], 9. Speed_Radial [m/s], 10. Azimuth_Angle [°], 11. Azimuth_Angle_rad [rad], 12. RCS [dB], 13. Signal_Level [dB],
# 14. Noise [dB], 15. Elevation_Angle [°], 16. Elevation_Angle_rad [rad], 17. Device_ID, 18. PortVersionMajor, 19. PortVersionMinor,
# 20. NTP_TimeStamp [µs], 21. BodyByteOrder, 22. PortIndex, 23. HeaderVersionMajor, 24. HeaderVersionMinor, 25. NofObjects,
# 26. CycleTime, 27. #, 28. Range [m], 29. Speed_Radial [m/s], 30. Azimuth [rad], 31. Elevation [rad], 32. Flags,
# 33. Device_ID, 34. UAT_Format_Version, 35. Source_Device_ID, 36. Nof_Instructions, 37. CRC, 38. Instruction_Counter,
# 39. Message_Type, 40. UAT_ID, 41. Par_No, 42. Result, 43. i32_value, 44. f_ieee754_value, 45. Data_Format,
# 46. Index1, 47. Index2, 48. Device_ID, 49. PortVersionMajor, 50. PortVersionMinor, 51. NTP_TimeStamp [µs],
# 52. BodyByteOrder, 53. PortIndex, 54. HeaderVersionMajor, 55. HeaderVersionMinor, 56. Source_Device_ID,
# 57. Nof_Instructions, 58. SequenceCounter, 59. EndTrigger, 60. Instruction_Counter, 61. InstructionRequest,
# 62. InstructionResponse, 63. Section, 64. ID, 65. Data_Type, 66. Dimension, 67. DimElemOf0, 68. DimElemOf1,
# 69. Signature, 70. u32_value, 71. i32_value, 72. f32_value, 73. u16_value, 74. i16_value, 75. u8_value,
# 76. i8_value, 77. u64_value,
# 113564590.5995 1135645905995 0 0.054976 21 8913 0 4.68 0 12.48 0.217776 -14 130 95 -8.08 -0.140996 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
# 113564590.7243 1135645907243 0 0.054976 21 8913 1 5.68 0 10.72 0.187064 13 159 95 -1.76 -0.030712 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: aclous_meas_to_bag.py csv-file output-bag-file start_time")
    else:
        if len(sys.argv) < 4:
            est_time = 1632412800.0  # Unix time, just, a guess :/
            time_offset = 2.5  # -2.7  # Seconds
            est_time = est_time + time_offset
            #          1632478228 # seconds
        else:
            est_time = sys.argv[3]

        df = pd.read_csv(sys.argv[1], delim_whitespace=True, header=None)
        # df = pd.read_csv("~/Meas_2021-09-23/meas008_UMRR_ID0_Targets_Port066_ID0_UAT_v4_Response_ID0_UAT_v4_InstructionPortResponse_InstructionPort.csv", delim_whitespace=True, header=None)
        last_tstmp = df[0][0]

        # Fenceposting, also done in the "if" statement
        point_list = []
        channel_list = []
        speed_radials = []

        with rosbag.Bag(sys.argv[2], "w") as bag:
            for row in range(df.shape[0]):
                timestamp = rospy.Time.from_sec(est_time)
                est_time = est_time + ((df[0][row] - last_tstmp) / 1000.0)  # seconds
                last_tstmp = df[0][row]

                # 8. Range [m], 9. Speed_Radial [m/s], 10. Azimuth_Angle [°], 11. Azimuth_Angle_rad [rad], 12. RCS [dB], 13. Signal_Level [dB],
                # 14. Noise [dB], 15. Elevation_Angle [°], 16. Elevation_Angle_rad [rad]
                range = df[7][row]  # m
                azimuth = df[10][row]  # rads
                elevation = df[15][row]  # rads

                x = math.cos(elevation) * math.cos(azimuth) * range  # forward
                y = math.cos(elevation) * math.sin(azimuth) * range  # left
                z = math.sin(elevation) * math.cos(azimuth) * range  # up
                new_point = Point32(x, y, z)
                point_list.append(new_point)

                speed_radials.append(df[8][row])

                # We found all the points of a single measurement, save meas and reset list
                if df[6][row] == 0:
                    # Prepare channel metadata
                    new_channel = ChannelFloat32()
                    new_channel.name = "speed_radial"
                    new_channel.values = speed_radials
                    channel_list.append(new_channel)
                    # Prepare message
                    pc_msg = PointCloud()
                    pc_msg.header.stamp = timestamp
                    pc_msg.header.frame_id = "radar_link"
                    pc_msg.channels = channel_list
                    pc_msg.points = point_list
                    # Save message
                    bag.write("/radar/pointcloud", pc_msg, timestamp)

                    # Reset things
                    speed_radials = []
                    point_list = []
                    channel_list = []
        print("Done saving bag")
