#include "ros/ros.h"
#include "robertbot_controller/AnglesConverter.h"
#include "math.h"

bool convert_radians_to_degrees(robertbot_controller::AnglesConverter::Request  &req,
         robertbot_controller::AnglesConverter::Response &res)
{
    // Function that is called every time the service radians_to_degrees is called
    // It receives the Request message as input with the angles in radians and the Result message Type as input 
    // it fills out the response message with the converted values 
    // and returns the Result message as output with the angles in degrees
    //res.base = static_cast<int>(((req.base+(M_PI/2))*180)/M_PI);
    //res.shoulder = 180-static_cast<int>(((req.shoulder+(M_PI/2))*180)/M_PI);
    //res.elbow = static_cast<int>(((req.elbow+(M_PI/2))*180)/M_PI);
    //res.gripper = static_cast<int>(((-req.gripper)*180)/(M_PI/2));
    res.base = static_cast<int>(((req.base+(M_PI/2))*180)/M_PI);
    res.body = static_cast<int>(((req.body+(M_PI/2))*180)/M_PI);
    res.longu = static_cast<int>(((req.longu+(M_PI/2))*180)/M_PI);
    res.coreu = static_cast<int>(((req.coreu+(M_PI/2))*180)/M_PI);
    res.claw = static_cast<int>(((req.claw+(M_PI/2))*180)/M_PI);
    res.gripper = static_cast<int>(((req.gripper+(M_PI/2))*180)/M_PI);
    return true;
}

bool convert_degrees_to_radians(robertbot_controller::AnglesConverter::Request  &req,
         robertbot_controller::AnglesConverter::Response &res)
{
    // Function that is called every time the service radians_to_degrees is called
    // It receives the Request message with the angles in degrees and the Result message Type as input 
    // it fills out the response message with the converted values 
    // and returns the Result message as output with the angles in radians
    //res.base = ((M_PI*req.base) - ((M_PI/2)*180))/180;
    //res.shoulder = (((180-req.shoulder)*M_PI)-((M_PI/2)*180))/180;
    //res.elbow = ((M_PI*req.elbow) - ((M_PI/2)*180))/180;
    //res.gripper = -((M_PI/2)*req.gripper)/180;
    res.base = ((M_PI*req.base) - ((M_PI/2)*180))/180;
    res.body = ((M_PI*req.body) - ((M_PI/2)*180))/180;
    res.longu = ((M_PI*req.longu) - ((M_PI/2)*180))/180;
    res.coreu = ((M_PI*req.coreu) - ((M_PI/2)*180))/180;
    res.claw = ((M_PI*req.claw) - ((M_PI/2)*180))/180;
    res.gripper = ((M_PI*req.gripper) - ((M_PI/2)*180))/180;
    return true;
}

int main(int argc, char **argv)
{
    // Inizialize a ROS node called angles_converter
    ros::init(argc, argv, "AngleConverter");
    ros::NodeHandle n;

    // Inizialize two services for the angle conversions 
    ros::ServiceServer radians_to_degrees = n.advertiseService("radians_to_degrees", convert_radians_to_degrees);
    ros::ServiceServer degrees_to_radians = n.advertiseService("degrees_to_radians", convert_degrees_to_radians);
    
    ROS_INFO("Angles Converter Service Started");

    // keeps the node up and running
    ros::spin();
    return 0;
}
