#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


class OdometryCalculator:
    def __init__(self):
        rospy.init_node("odometry_calculator")  # Initialize the ROS node
        self.odom_pub = rospy.Publisher(
            "odom", Odometry, queue_size=10
        )  # Publisher for odometry data
        self.twist_sub = rospy.Subscriber(
            "your_new_topic", Twist, self.twist_callback
        )  # Subscriber for Twist data

        self.x = 0.0  # Initial x coordinate
        self.y = 0.0  # Initial y coordinate
        self.theta = 0.0  # Initial orientation
        self.prev_time = rospy.Time.now()  # Initial timestamp

    def twist_callback(self, twist):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        dx = twist.linear.x * dt
        dy = twist.linear.y * dt
        dtheta = twist.angular.z * dt

        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
        self.theta += dtheta

        self.publish_odometry()

        self.prev_time = current_time

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quat = quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Publish the odometry message
        self.odom_pub.publish(odom)


if __name__ == "__main__":
    try:
        OdometryCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
