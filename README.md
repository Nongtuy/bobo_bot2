# small_robot_command_ws
Project focuses on communication between multiple robots.
Clone Najaaaa
```git clone https://github.com/Nongtuy/bobo_bot2.git```
# Systems
The robot's main systems are:
1. Differential drive(Track Based)
2. Lidar
# Packages
## 1. Obstacle_Avoidance-ROS
## 2. Communication_test
 2.1 Speaker.py This will be used to test communication for multimachine operation
 ```rosrun Communication_test Speaker.py```
## 3. movement_used
Folder [sketch_apr26a] is cpp arduino based, for low-level control. Both Turtlebot3_teleop and teleop_twist_keyboard can be used but   depend on hardwares behavior. high_level_type_3_2.py is the main high-level code. In order to use this code Lidar sensor must be activated.


command to be used
    
    ```rosrun movement_used high_level_type_3_2.py``` 
## 4. robot_1
## 5. rospy
## 6. sensor_node
## 7. sshping
## 8. teleop_twist_keyboard
