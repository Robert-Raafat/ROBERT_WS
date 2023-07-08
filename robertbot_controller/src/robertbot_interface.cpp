
#include "robertbot_controller/robertbot_interface.h"
#include <std_msgs/UInt16MultiArray.h>
#include "robertbot_controller/AnglesConverter.h"


robertbotInterface::robertbotInterface(ros::NodeHandle& nh) : nh_(nh), 
            pnh_("~"),
            pos_(6, 0),
            vel_(6, 0),
            eff_(6, 0),
            cmd_(6, 0),
            names_{"joint4", "joint7", "joint10", "joint13","joint18","joint22"}
{
    // Read from the param server
    pnh_.param("joint_names", names_, names_);

    // Init the publisher with the hardware
    hardware_pub_ = pnh_.advertise<std_msgs::UInt16MultiArray>("/robert/arm_actuate", 1000);
    hardware_srv_ = pnh_.serviceClient<robertbot_controller::AnglesConverter>("/radians_to_degrees");
    
    ROS_INFO("Starting robertbot Hardware Interface...");

    // connect and register joint state interface
    hardware_interface::JointStateHandle state_handle1(names_.at(0), &pos_.at(0), &vel_.at(0), &eff_.at(0));
    joint_state_interface_.registerHandle(state_handle1);
    hardware_interface::JointStateHandle state_handle2(names_.at(1), &pos_.at(1), &vel_.at(1), &eff_.at(1));
    joint_state_interface_.registerHandle(state_handle2);
    hardware_interface::JointStateHandle state_handle3(names_.at(2), &pos_.at(2), &vel_.at(2), &eff_.at(2));
    joint_state_interface_.registerHandle(state_handle3);
    hardware_interface::JointStateHandle state_handle4(names_.at(3), &pos_.at(3), &vel_.at(3), &eff_.at(3));
    joint_state_interface_.registerHandle(state_handle4);
    hardware_interface::JointStateHandle state_handle5(names_.at(4), &pos_.at(4), &vel_.at(4), &eff_.at(4));
    joint_state_interface_.registerHandle(state_handle5);
    hardware_interface::JointStateHandle state_handle6(names_.at(5), &pos_.at(5), &vel_.at(5), &eff_.at(5));
    joint_state_interface_.registerHandle(state_handle6);

    registerInterface(&joint_state_interface_);

    // connect and register joint position interface
    // the motors accept position inputs
    hardware_interface::JointHandle position_handle1(joint_state_interface_.getHandle(names_.at(0)), &cmd_.at(0));
    joint_position_interface_.registerHandle(position_handle1);
    hardware_interface::JointHandle position_handle2(joint_state_interface_.getHandle(names_.at(1)), &cmd_.at(1));
    joint_position_interface_.registerHandle(position_handle2);
    hardware_interface::JointHandle position_handle3(joint_state_interface_.getHandle(names_.at(2)), &cmd_.at(2));
    joint_position_interface_.registerHandle(position_handle3);
    hardware_interface::JointHandle position_handle4(joint_state_interface_.getHandle(names_.at(3)), &cmd_.at(3));
    joint_position_interface_.registerHandle(position_handle4);
    hardware_interface::JointHandle position_handle5(joint_state_interface_.getHandle(names_.at(4)), &cmd_.at(4));
    joint_position_interface_.registerHandle(position_handle5);
    hardware_interface::JointHandle position_handle6(joint_state_interface_.getHandle(names_.at(5)), &cmd_.at(5));
    joint_position_interface_.registerHandle(position_handle6);

    registerInterface(&joint_position_interface_);

    ROS_INFO("Interfaces registered.");


    ROS_INFO("Preparing the Controller Manager");

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    update_freq_ = ros::Duration(0.1);
    looper_ = nh_.createTimer(update_freq_, &robertbotInterface::update, this);
    
    ROS_INFO("Ready to execute the control loop");
}

void robertbotInterface::update(const ros::TimerEvent& e)
{
    // This function is called periodically in order to update the controller
    // manager about the progress in the execution of the goal of the hardware
    ROS_INFO("Update Event");
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void robertbotInterface::read()
{
    // Reads the current status of the Hardware (roberto)
    // Open Loop Control - no sensor available on the robot taht detects the effective
    // angle of totation of each joint. Suppose that the motors are always able to follow
    // the position command
    pos_.at(0) = cmd_.at(0);
    pos_.at(1) = cmd_.at(1);
    pos_.at(2) = cmd_.at(2);
    pos_.at(3) = cmd_.at(3);
    pos_.at(4) = cmd_.at(4);
    pos_.at(5) = cmd_.at(5);
}

void robertbotInterface::write(ros::Duration elapsed_time)
{    
    // Send the command to the Hardware (roberto)
    // First converts the angle from the moveit/urdf convention 
    // to the roberto convention and then publishes the converted angles
    robertbot_controller::AnglesConverter srv;
    srv.request.base = cmd_.at(0);
    srv.request.body = cmd_.at(1);
    srv.request.longu = cmd_.at(2);
    srv.request.coreu = cmd_.at(3);
    srv.request.claw = cmd_.at(4);
    srv.request.gripper = cmd_.at(5);

    // Call the service and show the response of the service
    if (hardware_srv_.call(srv))
    {
        // compose the array message
        std::vector<unsigned int> angles_deg;
        angles_deg.push_back(srv.response.base);
        angles_deg.push_back(srv.response.body);
        angles_deg.push_back(srv.response.longu);
        angles_deg.push_back(srv.response.coreu);
        angles_deg.push_back(srv.response.claw);
        angles_deg.push_back(srv.response.gripper);

        std_msgs::UInt16MultiArray msg;

        // set up dimensions
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = angles_deg.size();
        msg.layout.dim[0].stride = 1;

        // copy in the data
        msg.data.clear();
        msg.data.insert(msg.data.end(), angles_deg.begin(), angles_deg.end());

        // publish the array message to the defined topic
        hardware_pub_.publish(msg);
    }
    else
    {
        ROS_ERROR("Failed to call service radians_to_degrees");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robertbot_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    robertbotInterface robot(nh);

    // Keep ROS up and running
    spinner.spin();
    return 0;
}
