#include "segway_rmp200.h"
#include "ftdiexceptions.h"
#include "ftdiserver.h"
#include "ftdimodule.h"
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

std::string segway_name="segway";
CSegwayRMP200 *segway;
bool has_been_commanded = false;
int segway_motor_timeout = 0.5;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    has_been_commanded = true;
    segway->move(msg->linear.x, msg->angular.z);
    n.createTimer(ros::Duration(segway_motor_timeout), motor_timeoutCallback, true);
}

void statusCallback(const ros::TimerEvent& e) {
    std::stringstream ss;
    ss << std::endl << (*segway);
    ROS_INFO(ss.str().c_str());
}

void motor_timeoutCallback(const ros::TimerEvent& e) {
    if(has_been_commanded) { // If it has been commanded, note that it has been checked but not stopped
        has_been_commanded = false;
        has_been_stopped = false;
    } else if(!has_been_stopped) { // If it hasn't been commanded since and it hasn't been stopped yet, stop it
        ROS_WARN("Motor timeout reached, stopping segway.");
        has_ben_stopped = true;
        segway->stop();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "segway_rmp200");

    ros::NodeHandle n;

    ROS_INFO("Setting up Segway Interface.");

    CFTDIServer *ftdi_server=CFTDIServer::instance();
    std::string serial_number;

    try {
        ftdi_server->add_custom_PID(0xE729);
        if(ftdi_server->get_num_devices()>0) {
            serial_number=ftdi_server->get_serial_number(0);
            segway=new CSegwayRMP200(segway_name);
            segway->connect(serial_number);
            segway->unlock_balance();
            segway->set_operation_mode(balance);
            segway->set_gain_schedule(light);
            segway->reset_right_wheel_integrator();
            usleep(10000);
            segway->reset_left_wheel_integrator();
            usleep(10000);
            segway->reset_yaw_integrator();
            usleep(10000);
            segway->reset_forward_integrator();
            usleep(10000);
        }
    } catch(CException &e) {
        ROS_ERROR(e.what().c_str());
        ROS_WARN("It seems like there was an error connecting to the segway, check your connections, permissions, and that the segway powerbase is on.");
        exit(-1);
    }

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);

    ROS_INFO("Segway Ready.");

    // Setup Status Loop
    int segway_status_rate;
    n.param("segway_status_rate", segway_status_rate, 2);
    ros::Timer status_timer = n.createTimer(ros::Duration(1.0/segway_status_rate), statusCallback);

    // Setup Motor Timeout
    n.param("segway_motor_timeout", segway_motor_timeout, 0.5);

    ros::spin();

    ROS_INFO("Shutting Down Segway Interface.");
    segway->stop();
    // segway->close();  // This seems to hang the program on exit, need to look into that...

    return 0;
}