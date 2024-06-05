# bobo_bot2
Project focuses on communication between multiple robots.
#### Clone Najaaaa

```git clone https://github.com/Nongtuy/bobo_bot2.git```

#### source workspace

```cd bobo_bot2```

```source devel/setup.bash```
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
Robot_1 is simulation based on rviz and gazebo to test deff-drive
##### Note: Flip robot back before using
```roslauch robot_1 gazebo.launch```
## 5. rospy(Repair Package during development)

## 6. sensor_node
Sensor_node is subscriber for sensor infors
## 7. sshping(Naaaaaaaaaa)
## 8. teleop_twist_keyboard
Use for manual control

 ```rosrun teleop_twist_keyboard```
 # How To Use??????????
This package is made on Rpi 4 B to arduino to control the robot. You need to upload sketch_apr26a to your arduino board. Topics can be added for more complex robot. 

Login SSH to Rpi board then.

  ```rosrun rosserial_python serial_node.py  _port:=/dev/ttyACM0 _baud:=115200```

  To change node name you can use

```rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200 _name:=Your_desire_name```
  
