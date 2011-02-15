#include "segway_rmp400.h"
#include "ftdiexceptions.h"
#include "ftdiserver.h"
#include "ftdimodule.h"
#include <iostream>
#include <sstream>
#include <math.h>

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "segway_rmp400/SegwayStatus400.h"

#define pi 3.14159265

// Segway Interface Variables
std::string segway_name="segway";
CSegwayRMP200 *segway;

// Persistent Odometry Variables
bool first_odometry = true;
float last_forward_displacement = 0, last_yaw_displacement = 0;
float odometry_x = 0.0, odometry_y = 0.0, odometry_w = 0.0;
ros::Time last_time;

// ROS variables
ros::NodeHandle *n;
std::string frame_id = "base_link";
double segway_motor_timeout = 0.5;
ros::Publisher segway_status_pub;
ros::Publisher odom_pub;
tf::TransformBroadcaster *odom_broadcaster;
ros::Timer motor_timeout_timer;

void odometryCallback(const ros::TimerEvent& e) {
    // Grab the time
    ros::Time current_time = ros::Time::now();
    
    // Grab the newest Segway data
    float forward_displacement = segway->get_forward_displacement(); // Meters
    float yaw_displacement = segway->get_yaw_displacement()*2*pi; // Radians
    float yaw_rate = segway->get_yaw_rate()*(pi/180.0); // radians/s
    
    // Integrate the displacements over time
    // If not the first odometry calculate the delta in displacements
    float vel_x = 0.0;
    float vel_y = 0.0;
    if(!first_odometry) {
        float delta_forward_displacement = forward_displacement - last_forward_displacement;
        double delta_time = (current_time-last_time).toSec();
        // Update accumulated odometries and calculate the x and y components of velocity
        odometry_w = yaw_displacement;
        float new_odometry_x = delta_forward_displacement * std::cos(odometry_w);
        vel_x = (new_odometry_x - odometry_x)/delta_time;
        odometry_x += new_odometry_x;
        float new_odometry_y = delta_forward_displacement * std::sin(odometry_w);
        vel_y = (new_odometry_y - odometry_y)/delta_time;
        odometry_y += new_odometry_y;
    } else {
        first_odometry = false;
    }
    // No matter what update the previouse (last) displacements
    last_forward_displacement = forward_displacement;
    last_yaw_displacement = yaw_displacement;
    last_time = current_time;
    
    // Create a Quaternion from the yaw displacement
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw_displacement);
    
    // Publish the Transform odom->base_link
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = frame_id;

    odom_trans.transform.translation.x = odometry_x;
    odom_trans.transform.translation.y = odometry_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quat;

    //send the transform
    odom_broadcaster->sendTransform(odom_trans);
    
    // Publish Odometry
    nav_msgs::Odometry msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "odom";
    msg.pose.pose.position.x = odometry_x;
    msg.pose.pose.position.y = odometry_y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = quat;
    
    msg.child_frame_id = frame_id;
    msg.twist.twist.linear.x = vel_x;
    msg.twist.twist.linear.y = vel_y;
    msg.twist.twist.angular.z = yaw_rate;
    
    odom_pub.publish(msg);
}

void statusCallback(const ros::TimerEvent& e) {
    segway_rmp200::SegwayStatus msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    
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
    segway->stop();
}

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    segway->move(msg->linear.x, msg->angular.z);
    motor_timeout_timer = n->createTimer(ros::Duration(segway_motor_timeout), motor_timeoutCallback, true);
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
        ROS_ERROR("%s", e.what().c_str());
        ROS_WARN("It seems like there was an error connecting to the segway, check your connections, permissions, and that the segway powerbase is on.");
        exit(-1);
    }
    
    ros::Subscriber sub = n->subscribe("cmd_vel", 1000, cmd_velCallback);
    
    ROS_INFO("Segway Ready.");
    
    // Setup Status Loop
    int segway_status_rate;
    n->param("segway_status_rate", segway_status_rate, 2);
    ros::Timer status_timer = n->createTimer(ros::Duration(1.0/segway_status_rate), statusCallback);
    
    // Setup Odometry Loop
    int segway_odom_rate;
    n->param("segway_odom_rate", segway_odom_rate, 30);
    ros::Timer odom_timer = n->createTimer(ros::Duration(1.0/segway_odom_rate), odometryCallback);
    
    // Setup Motor Timeout
    n->param("segway_motor_timeout", segway_motor_timeout, 0.5);
    
    // Get frame id parameter
    n->param("frame_id", frame_id, std::string("base_link"));
    
    // Setup the Segway Status Publisher
    segway_status_pub = n->advertise<segway_rmp200::SegwayStatus>("segway_status", 1000);
    
    // Setup the Odometry Publisher
    odom_pub = n->advertise<nav_msgs::Odometry>("odom", 50);
    
    // Setup the TF broadcaster
    odom_broadcaster = new tf::TransformBroadcaster;
    
    ros::spin();
    
    ROS_INFO("Shutting Down Segway Interface.");
    segway->stop();
    delete ((ros::Timer)motor_timeout_timer);
    // segway->close();  // This seems to hang the program on exit, need to look into that...
    
    return 0;
}
