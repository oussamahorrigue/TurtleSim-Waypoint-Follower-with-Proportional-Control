#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')

        self.pose = Pose()
        self.target_poses = [
            (1.8, 3.2),   # Waypoint 1: (x, y) = (1.8, 3.2)
            (2.5, 0.2),   # Waypoint 2: (x, y) = (2.5, 0.2)
            (3.8, 0.6),   # Waypoint 3: (x, y) = (3.8, 0.6)
            (2.0, 4.0)    # Waypoint 4: (x, y) = (2.0, 4.0)
        ]
        self.current_waypoint = 0
        self.stay_durations = [1, 2, 3, 4]  # Stay durations at each waypoint (in seconds)

        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

    def pose_callback(self, data):
        self.pose = data

    def calculate_distance(self):
        target_pose = self.target_poses[self.current_waypoint]
        return math.sqrt((target_pose[0] - self.pose.x)**2 + (target_pose[1] - self.pose.y)**2)

    def reached_waypoint(self):
        target_pose = self.target_poses[self.current_waypoint]
        distance_threshold = 0.1
        return self.calculate_distance() < distance_threshold

    def stay_at_waypoint(self):
        stay_duration = self.stay_durations[self.current_waypoint]
        rospy.loginfo("Staying at waypoint {} for {} seconds".format(self.current_waypoint + 1, stay_duration))
        rospy.sleep(stay_duration)

    def move_to_next_waypoint(self):
        self.current_waypoint = (self.current_waypoint + 1) % len(self.target_poses)

    def run(self):
        while not rospy.is_shutdown():
            if self.reached_waypoint():
                rospy.loginfo("Reached waypoint {}".format(self.current_waypoint + 1))
                self.stay_at_waypoint()
                self.move_to_next_waypoint()

            target_pose = self.target_poses[self.current_waypoint]

            # Calculate linear and angular velocities
            distance = self.calculate_distance()
            angle = math.atan2(target_pose[1] - self.pose.y, target_pose[0] - self.pose.x)
            angular_error = angle - self.pose.theta

            # Normalize the angular error to be within -pi to pi
            if angular_error > math.pi:
                angular_error -= 2 * math.pi
            elif angular_error < -math.pi:
                angular_error += 2 * math.pi

            linear_vel = 0.5 * distance
            angular_vel = 4.0 * angular_error

            # Create Twist message and publish it
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            self.pub.publish(twist)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = WaypointFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass

