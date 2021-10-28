#!/usr/bin/env python
# import roslib
# roslib.load_manifest('learning_tf')
import rospy

from sensor_msgs.msg import Imu, NavSatFix


def change_covariance_imu(msg, pub):
    cov = 1.0
    msg.orientation_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov]
    msg.angular_velocity_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov]
    msg.linear_acceleration_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov]
    pub.publish(msg)


def change_covariance_gps(msg, pub):
    # msg = NavSatFix()  # Intellisense temp
    cov = 2
    msg.position_covariance = [cov, 0, 0,
                               0, cov, 0,
                               0, 0, cov]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    msg.altitude = 0.0  # force
    pub.publish(msg)


if __name__ == "__main__":
    pub_imu = rospy.Publisher("/imu/data_extracov", Imu, queue_size=1)
    pub_gps = rospy.Publisher("/gps/fix", NavSatFix, queue_size=1)
    rospy.init_node("imu_covariance_inflation")
    rospy.Subscriber("/imu/data", Imu, change_covariance_imu, pub_imu)
    rospy.Subscriber("/gps/raw", NavSatFix, change_covariance_gps, pub_gps)
    print("Started adding covariance")
    rospy.spin()
