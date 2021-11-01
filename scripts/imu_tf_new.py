#!/usr/bin/env python
import rospy

import tf
from sensor_msgs.msg import Imu


def handle_imu_transform(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform(
        (0, 0, 0),  # Translation
        tf.transformations.quaternion_from_euler(0, 0, 0),  # Rotation
        msg.header.stamp,
        "base_link",
        "imu_link",
    )


if __name__ == "__main__":
    rospy.init_node("imu_tf_broadcaster")
    rospy.Subscriber("/imu/data_extracov", Imu, handle_imu_transform)
    print("Generating new tf")
    rospy.spin()
