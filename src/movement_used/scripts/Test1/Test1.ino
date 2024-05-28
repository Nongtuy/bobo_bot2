#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// Motor A connections
int enA = 46;
int in1 = 1;
int in2 = 2;
// Motor B connections
int enB = 44;
int in3 = 3;
int in4 = 4;

void setup() {
  // Initialize ROS
  nh.initNode();
  nh.advertise();
  
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Initialize subscriber to receive Twist messages
  nh.subscribe("/cmd_vel", &cmdVelCallback);
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();

  // Your existing loop code...
}

// Callback function for receiving Twist messages
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  // Extract linear and angular velocities from Twist message
  float linear_speed = msg.linear.x;
  float angular_speed = msg.angular.z;

  // Convert linear and angular speeds to differential speeds for left and right motors
  float left_speed = linear_speed - angular_speed;
  float right_speed = linear_speed + angular_speed;

  // Convert differential speeds to PWM signals for left and right motors
  int left_pwm = map(left_speed, -1, 1, -255, 255);
  int right_pwm = map(right_speed, -1, 1, -255, 255);

  // Set PWM signals for left motor
  if (left_pwm > 0) {
    analogWrite(enA, left_pwm);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    analogWrite(enA, -left_pwm);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  // Set PWM signals for right motor
  if (right_pwm > 0) {
    analogWrite(enB, right_pwm);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    analogWrite(enB, -right_pwm);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
}
