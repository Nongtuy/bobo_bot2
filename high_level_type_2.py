#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.move_cmd = Twist()
        self.turning = False
        self.turn_start_time = None

    def scan_callback(self, scan_msg):
        # Parameters for obstacle avoidance
        min_dist = 1.0  # Minimum distance to obstacle
        safe_distance = 2.0  # Safe distance to keep from obstacles
        max_linear_speed = 0.8  # Maximum linear speed
        turning_linear_speed = 0.26  # Linear speed while turning
        max_angular_speed = 0.5  # Maximum angular speed
        turn_duration = 0.1  # Duration of turning in seconds

        # Calculate angular speed based on the closest obstacle
        min_dist_index = min(range(len(scan_msg.ranges)), key=lambda i: scan_msg.ranges[i])
        closest_obstacle_dist = scan_msg.ranges[min_dist_index]

        if closest_obstacle_dist < min_dist:
            # Obstacle detected
            if not self.turning:
                self.turning = True
                self.turn_start_time = rospy.Time.now()

            if rospy.Time.now() - self.turn_start_time < rospy.Duration.from_sec(turn_duration):
                # Turn for turn_duration seconds
                self.move_cmd.linear.x = turning_linear_speed
                self.move_cmd.angular.z = 0.0
            else:
                # Stop turning, move forward slowly
                self.turning = False
                self.move_cmd.linear.x =  0.0
                self.move_cmd.angular.z = max_angular_speed
        else:
            # No obstacle, move forward with angular velocity -1.82
            self.move_cmd.linear.x = max_linear_speed
              # Publish the movement command
        self.pub.publish(self.move_cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        avoidance = ObstacleAvoidance()
        avoidance.run()
    except rospy.ROSInterruptException:
        pass
