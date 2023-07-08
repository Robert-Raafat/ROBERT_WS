/*
  arduinobot - ros_robot_control
  Script that creates a ROS node on the Arduino that subscribes to 
  joint angle messages and actuates the servo motors according to the
  selected angles
  
  Copyright (c) 2021 Antonio Brandi.  All right reserved.
*/

#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  105 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Define Motor Outputs on PCA9685 board
#define motorBase 0//from 0 to 180 but 80 is the optimum
#define motorSholder 1//from 15 to 180 but 0 is the optimum
#define motorElbow 2//from 0 to 180 but 0 is the optimum
#define motorWrist 3//from 0 to 180 but 10 is the optimum
#define gripper_angel 4//from 0 to 180 but 90 is the optimum
#define gripper 5 //Open Angle 85 deg ,close angle 135 deg
#define MIN_RANGE 0
#define MAX_RANGE 180


// Register the servo motors of each joint
//Servo motor;    

// Initialize the ROS node
ros::NodeHandle  nh;

/*
 * This function is called each time a new message is published on the topic /servo_actuate
 */
void arm_actuate_cb( const std_msgs::UInt16MultiArray& msg){
  
  if(msg.data[0]<MIN_RANGE) msg.data[0] = MIN_RANGE;
  if(msg.data[1]<15) msg.data[1] = 15;
  if(msg.data[2]<MIN_RANGE) msg.data[2] = MIN_RANGE;
  if(msg.data[3]<MIN_RANGE) msg.data[3] = MIN_RANGE;
  if(msg.data[4]<MIN_RANGE) msg.data[4] = MIN_RANGE;
  if(msg.data[5]<85) msg.data[5] = 85;

  if(msg.data[0]>MAX_RANGE) msg.data[0] = MAX_RANGE;
  if(msg.data[1]>MAX_RANGE) msg.data[1] = MAX_RANGE;
  if(msg.data[2]>MAX_RANGE) msg.data[2] = MAX_RANGE;
  if(msg.data[3]>MAX_RANGE) msg.data[3] = MAX_RANGE;
  if(msg.data[4]>MAX_RANGE) msg.data[4] = MAX_RANGE;
  if(msg.data[5]>135) msg.data[5] = 135;

  moveMotorDeg(msg.data[0],motorBase);
  moveMotorDeg(msg.data[1],motorSholder);
  moveMotorDeg(msg.data[2],motorElbow);
  moveMotorDeg(msg.data[3],motorWrist);
  moveMotorDeg(msg.data[4],gripper_angel);
  moveMotorDeg(msg.data[5],gripper);
  delay(5);
}

// Define the subscriber to the topic /servo_actuate where are published UInt16MultiArray messages
// Define the function that will be triggered each time a new message is published on this topic
ros::Subscriber<std_msgs::UInt16MultiArray> sub("robert/arm_actuate", &arm_actuate_cb );


void setup() {
  // Attach and Initialize each Servo to the Arduino pin where it is connected

  // Set a common start point for each joint
  // This way, the start status of each joint is known

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  moveMotorDeg(90,motorBase);
  moveMotorDeg(45,motorSholder);
  moveMotorDeg(30,motorElbow);
  moveMotorDeg(120,motorWrist);
  moveMotorDeg(90,gripper_angel);
  moveMotorDeg(85,gripper);
  // Inizialize the ROS node on the Arduino
  nh.initNode();
  // Inform ROS that this node will subscribe to messages on a given topic
  nh.subscribe(sub);
}

void moveMotorDeg(int moveDegree, int motorOut)
  {
    int pulse_width;
    // Convert to pulse width
    pulse_width = map(moveDegree, 0, 180, SERVOMIN, SERVOMAX);

    //Control Motor
    pwm.setPWM(motorOut, 0, pulse_width);
  }

void loop() {
  // Keep the ROS node up and running
  nh.spinOnce();
  delay(1);
}
