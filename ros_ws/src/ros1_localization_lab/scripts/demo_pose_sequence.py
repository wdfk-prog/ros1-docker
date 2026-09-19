#!/usr/bin/env python3

import math
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


def make_pose(x_m, y_m, yaw_deg):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.pose.pose.position.x = x_m
    msg.pose.pose.position.y = y_m

    yaw_rad = math.radians(yaw_deg)
    msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

    msg.pose.covariance[0] = 0.01
    msg.pose.covariance[7] = 0.01
    msg.pose.covariance[35] = math.radians(1.0) ** 2
    return msg


def main():
    rospy.init_node("demo_pose_sequence_publisher")
    pub = rospy.Publisher("/demo_pose", PoseWithCovarianceStamped, queue_size=10)

    deadline = time.monotonic() + 2.0
    while pub.get_num_connections() == 0 and time.monotonic() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)

    sequence = [
        (10.0, 2.0, 30.0),
        (10.5, 2.1, 32.0),
        (11.2, 2.2, 35.0),
    ]

    for x_m, y_m, yaw_deg in sequence:
        if rospy.is_shutdown():
            break
        msg = make_pose(x_m, y_m, yaw_deg)
        pub.publish(msg)
        rospy.loginfo("published demo pose: x=%.3f y=%.3f yaw=%.1f deg", x_m, y_m, yaw_deg)
        rospy.sleep(1.0)


if __name__ == "__main__":
    main()
