#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def publisher_node():
    # Initialize the node
    rospy.init_node('cmd_vel_publisher', anonymous=True)
    
    # Create a publisher for the cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Set the rate at which to publish messages (10 Hz in this case)
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a Twist message to publish velocity commands
    twist_msg = Twist()
    twist_msg.linear.x = 0.1  # Set linear velocity (m/s)
    twist_msg.angular.z = 0.5  # Set angular velocity (rad/s)
    
    while not rospy.is_shutdown():
        # Publish the Twist message
        pub.publish(twist_msg)
        
        # Log the published message
        rospy.loginfo("Publishing Twist message: linear=%f, angular=%f",
                      twist_msg.linear.x, twist_msg.angular.z)
        
        # Sleep to maintain the desired publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        # Run the publisher node
        publisher_node()
    except rospy.ROSInterruptException:
        pass
