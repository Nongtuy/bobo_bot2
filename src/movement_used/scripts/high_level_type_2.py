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
        # Define the angle range for the front view in radians
        self.front_view_angle_min = -math.pi / 6  # -45 degrees
        self.front_view_angle_max = math.pi / 6   # 45 degrees

    def scan_callback(self, scan_msg):
        # Parameters for obstacle avoidance
        min_dist = 0.3  # Minimum distance to obstacle
        safe_distance = 2.0  # Safe distance to keep from obstacles
        max_linear_speed = 0.26  # Maximum linear speed
        turning_linear_speed = 0.26  # Linear speed while turning
        max_angular_speed = -1.82  # Maximum angular speed
        turn_duration = 1.0  # Duration of turning in seconds

        # Filter the laser scan data to focus on the front view
        front_view_ranges = scan_msg.ranges[int((self.front_view_angle_min - scan_msg.angle_min) / scan_msg.angle_increment):
                                             int((self.front_view_angle_max - scan_msg.angle_min) / scan_msg.angle_increment)]

        # Calculate angular speed based on the closest obstacle in the front view
        closest_obstacle_dist = min(front_view_ranges)

        if closest_obstacle_dist < min_dist:
            # Obstacle detected
            if not self.turning:
                self.turning = True
                self.turn_start_time = rospy.Time.now()

            if rospy.Time.now() - self.turn_start_time < rospy.Duration.from_sec(turn_duration):
                # Turn for turn_duration seconds
                #self.move_cmd.linear.x = turning_linear_speed
                #self.move_cmd.angular.z = 0.0
                self.move_cmd.linear.x = 0.26
                self.move_cmd.angular.z = 0.0

            else:
                # Stop turning, move forward slowly
                self.turning = False
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.0
        else:
            # No obstacle, move forward with angular velocity -1.82
            self.move_cmd.angular.z = -1.82

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

