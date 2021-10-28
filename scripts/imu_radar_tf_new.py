#!/usr/bin/env python
import rospy

import tf
from sensor_msgs.msg import Imu


def handle_imu_radar_transform(msg):
    br = tf.TransformBroadcaster()
    tempo = msg.orientation
    rot = [tempo.x, tempo.y, tempo.z, tempo.w]
    br.sendTransform(
        (0, 0, 0),  # Translation
        rot,  # Rotation
        msg.header.stamp,
        "new_world",
        "radar1",
    )


if __name__ == "__main__":
    rospy.init_node("imu_radar_tf_broadcaster")
    rospy.Subscriber("/imu/data", Imu, handle_imu_radar_transform)
    print("Generating new frame 'new_world'")
    rospy.spin()
