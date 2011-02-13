#include "segway_rmp200.h"
#include "ftdiexceptions.h"
#include "ftdiserver.h"
#include "ftdimodule.h"
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "segway_rmp200/SegwayStatus.h"

std::string segway_name="segway";
CSegwayRMP200 *segway;
bool has_been_commanded = false;
double segway_motor_timeout = 0.5;
bool has_been_stopped = false;
ros::NodeHandle *n;
std::string frame_id = "base_link";
ros::Publisher segway_status_pub;

void statusCallback(const ros::TimerEvent& e) {
    segway_rmp200::SegwayStatus msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    
    // Grab mutex for the status data
    msg.pitch_angle = segway->get_pitch_angle();
    msg.pitch_rate = segway->get_pitch_rate();
    msg.roll_angle = segway->get_roll_angle();
    msg.roll_rate = segway->get_roll_rate();
    msg.left_wheel_velocity = segway->get_left_wheel_velocity();
    msg.right_wheel_velocity = segway->get_right_wheel_velocity();
    msg.yaw_rate = segway->get_yaw_rate();
    msg.servo_frames = segway->get_servo_frames();
    msg.left_wheel_displacement = segway->get_left_wheel_displacement();
    msg.right_wheel_displacement = segway->get_right_wheel_displacement();
    msg.forward_displacement = segway->get_forward_displacement();
    msg.yaw_displacement = segway->get_yaw_displacement();
    msg.left_motor_torque = segway->get_left_motor_torque();
    msg.right_motor_torque = segway->get_right_motor_torque();
    msg.operation_mode = segway->get_operation_mode();
    msg.gain_schedule = segway->get_gain_schedule();
    msg.ui_battery = segway->get_ui_battery_voltage();
    msg.powerbase_battery = segway->get_powerbase_battery_voltage();
    
    segway_status_pub.publish(msg);
}

void motor_timeoutCallback(const ros::TimerEvent& e) {
    if(has_been_commanded) { // If it has been commanded, note that it has been checked but not stopped
        has_been_commanded = false;
        has_been_stopped = false;
    } else if(!has_been_stopped) { // If it hasn't been commanded since and it hasn't been stopped yet, stop it
        ROS_WARN("Motor timeout reached, stopping segway.");
        has_been_stopped = true;
        segway->stop();
    }
}

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    has_been_commanded = true;
    segway->move(msg->linear.x, msg->angular.z);
    n->createTimer(ros::Duration(segway_motor_timeout), motor_timeoutCallback, true);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "segway_rmp200");

    n = new ros::NodeHandle;

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

    ros::Subscriber sub = n->subscribe("cmd_vel", 1000, cmd_velCallback);

    ROS_INFO("Segway Ready.");

    // Setup Status Loop
    int segway_status_rate;
    n->param("segway_status_rate", segway_status_rate, 2);
    ros::Timer status_timer = n->createTimer(ros::Duration(1.0/segway_status_rate), statusCallback);

    // Setup Motor Timeout
    n->param("segway_motor_timeout", segway_motor_timeout, 0.5);
    
    // Get frame id parameter
    n->param("frame_id", frame_id, std::string("base_link"));
    
    // Setup the Segway Status Publisher
    segway_status_pub = n->advertise<segway_rmp200::SegwayStatus>("segway_status", 1000);

    ros::spin();

    ROS_INFO("Shutting Down Segway Interface.");
    segway->stop();
    // segway->close();  // This seems to hang the program on exit, need to look into that...

    return 0;
}
