#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')

        self.pose = Pose()
        self.target_pose = Pose()
        self.target_pose.x = rospy.get_param('~waypoint_x', 5.0)  # Default waypoint x-coordinate
        self.target_pose.y = rospy.get_param('~waypoint_y', 5.0)  # Default waypoint y-coordinate

        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.kp = 1.0  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.0  # Derivative gain
        self.prev_error = 0.0
        self.total_error = 0.0

    def pose_callback(self, data):
        self.pose = data

    def calculate_distance(self):
        return math.sqrt((self.target_pose.x - self.pose.x)**2 + (self.target_pose.y - self.pose.y)**2)

    def calculate_heading(self):
        return math.atan2(self.target_pose.y - self.pose.y, self.target_pose.x - self.pose.x)

    def calculate_error(self):
        target_heading = self.calculate_heading()
        current_heading = self.pose.theta

        # Ensure the error is within -pi to pi range
        error = math.atan2(math.sin(target_heading - current_heading), math.cos(target_heading - current_heading))
        return error

    def pid_control(self):
        error = self.calculate_error()

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.total_error += error
        i_term = self.ki * self.total_error

        # Derivative term
        d_term = self.kd * (error - self.prev_error)
        self.prev_error = error

        # Calculate control output
        control_output = p_term + i_term + d_term

        return control_output

    def run(self):
        while not rospy.is_shutdown():
            if self.calculate_distance() < 0.1:  # Check if reached target waypoint
                rospy.loginfo("Reached target waypoint")
                self.target_pose.x = rospy.get_param('~waypoint_x', 5.0)  # Reset to default waypoint x-coordinate
                self.target_pose.y = rospy.get_param('~waypoint_y', 5.0)  # Reset to default waypoint y-coordinate

            # Calculate control output
            control_output = self.pid_control()

            # Publish control output as Twist message
            twist = Twist()
            twist.linear.x = control_output
            twist.angular.z = 0.0  # Adjust if using angular control
            self.pub.publish(twist)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = WaypointFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass

