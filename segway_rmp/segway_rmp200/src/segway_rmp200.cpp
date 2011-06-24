/*
 * The MIT License (MIT)
 * Copyright (c) 2011 William Woodall <wjwwood@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include <sstream>
#include <cmath>

#include <boost/thread.hpp>

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "segway_rmp200/SegwayStatusStamped.h"

#include "segwayrmp.h"

class SegwayRMPNode;

static SegwayRMPNode * segwayrmp_node_instance;

static double degrees_to_radians = 180.0 / M_PI;

// Message Wrappers
void handleDebugMessages(const std::string &msg) {ROS_DEBUG("%s",msg.c_str());}
void handleInfoMessages(const std::string &msg) {ROS_INFO("%s",msg.c_str());}
void handleErrorMessages(const std::string &msg) {ROS_ERROR("%s",msg.c_str());}

void handleStatusWrapper(segwayrmp::SegwayStatus &ss);

// ROS Node class
class SegwayRMPNode {
public:
    SegwayRMPNode() {
        n = new ros::NodeHandle("~");
        this->segway_rmp = NULL;
        this->target_linear_vel = 0.0;
        this->first_odometry = false;
        this->last_forward_displacement = 0.0;
        this->last_yaw_displacement = 0.0;
        this->odometry_x = 0.0;
        this->odometry_y = 0.0;
        this->odometry_w = 0.0;
        this->count = 0;
    }
    
    ~SegwayRMPNode() {
        this->disconnect();
    }
    
    void disconnect() {
        if (this->segway_rmp != NULL)
            delete this->segway_rmp;
        this->segway_rmp = NULL;
    }
    
    void run() {
        if (this->getParameters()) {
            return;
        }
        
        this->setupSegwayRMP();
        
        this->setupROSComms();
        
        // Setup keep alive timer
        this->keep_alive_timer = n->createTimer(ros::Duration(1.0/20.0), &SegwayRMPNode::keepAliveCallback, this);
        
        this->connected = false;
        while (ros::ok()) {
            try {
                this->segway_rmp->connect();
                this->connected = true;
            } catch (std::exception& e) {
                std::string e_msg(e.what());
                ROS_ERROR("Exception while connecting to the SegwayRMP, check your cables and power buttons.");
                ROS_ERROR("    %s", e_msg.c_str());
                this->connected = false;
            }
            if (this->spin()) { // ROS is OK, but we aren't connected, wait then try again
                ROS_WARN("Not connected to the SegwayRMP, will retry in 5 seconds...");
                ros::Duration(5).sleep();
            }
        }
    }
    
    bool spin() {
        if (ros::ok() && this->connected) {
            ROS_INFO("Segway RMP Ready.");
            while (ros::ok() && this->connected) {
                ros::Duration(1.0).sleep();
            }
        }
        if (ros::ok()) { // Error not shutdown
            return true;
        } else {         // Shutdown
            return false;
        }
    }
    
    void keepAliveCallback(const ros::TimerEvent& e) {
        if (ros::ok()) {
            boost::mutex::scoped_lock lock(this->m_mutex);
            try {
                this->segway_rmp->move(this->linear_vel, this->angular_vel);
            } catch (std::exception& e) {
                std::string e_msg(e.what());
                ROS_ERROR("Error commanding Segway RMP: %s", e_msg.c_str());
                this->connected = false;
                this->disconnect();
            }
        }
    }
    
    void handleStatus(segwayrmp::SegwayStatus &ss) {
        // Get the time
        ros::Time current_time = ros::Time::now();
        
        this->sss_msg.header.stamp = current_time;
        
        this->sss_msg.segway.pitch_angle = ss.pitch * degrees_to_radians;
        this->sss_msg.segway.pitch_rate = ss.pitch_rate * degrees_to_radians;
        this->sss_msg.segway.roll_angle = ss.roll * degrees_to_radians;
        this->sss_msg.segway.roll_rate = ss.roll_rate * degrees_to_radians;
        this->sss_msg.segway.left_wheel_velocity = ss.left_wheel_speed;
        this->sss_msg.segway.right_wheel_velocity = ss.right_wheel_speed;
        this->sss_msg.segway.yaw_rate = ss.yaw_rate * degrees_to_radians;
        this->sss_msg.segway.servo_frames = ss.servo_frames;
        this->sss_msg.segway.left_wheel_displacement = ss.integrated_left_wheel_position;
        this->sss_msg.segway.right_wheel_displacement = ss.integrated_right_wheel_position;
        this->sss_msg.segway.forward_displacement = ss.integrated_forward_position;
        this->sss_msg.segway.yaw_displacement = ss.integrated_turn_position * degrees_to_radians;
        this->sss_msg.segway.left_motor_torque = ss.left_motor_torque;
        this->sss_msg.segway.right_motor_torque = ss.right_motor_torque;
        this->sss_msg.segway.operation_mode = ss.operational_mode;
        this->sss_msg.segway.gain_schedule = ss.controller_gain_schedule;
        this->sss_msg.segway.ui_battery = ss.ui_battery_voltage;
        this->sss_msg.segway.powerbase_battery = ss.powerbase_battery_voltage;
        this->sss_msg.segway.motors_enabled = (bool)(ss.motor_status);
        
        segway_status_pub.publish(this->sss_msg);
        
        // TODO: possibly spin this off in another thread
        
        // Grab the newest Segway data
        float forward_displacement = ss.integrated_forward_position;
        float yaw_displacement = ss.integrated_turn_position * degrees_to_radians;
        float yaw_rate = ss.yaw_rate * degrees_to_radians;
        
        // Integrate the displacements over time
        // If not the first odometry calculate the delta in displacements
        float vel_x = 0.0;
        float vel_y = 0.0;
        if(!this->first_odometry) {
            float delta_forward_displacement = 
                forward_displacement - this->last_forward_displacement;
            double delta_time = (current_time-this->last_time).toSec();
            // Update accumulated odometries and calculate the x and y components 
            // of velocity
            this->odometry_w = yaw_displacement;
            float new_odometry_x = 
                delta_forward_displacement * std::cos(this->odometry_w);
            vel_x = (new_odometry_x - this->odometry_x)/delta_time;
            this->odometry_x += new_odometry_x;
            float new_odometry_y = 
                delta_forward_displacement * std::sin(this->odometry_w);
            vel_y = (new_odometry_y - this->odometry_y)/delta_time;
            this->odometry_y += new_odometry_y;
        } else {
            this->first_odometry = false;
        }
        // No matter what update the previouse (last) displacements
        this->last_forward_displacement = forward_displacement;
        this->last_yaw_displacement = yaw_displacement;
        this->last_time = current_time;
        
        // Create a Quaternion from the yaw displacement
        geometry_msgs::Quaternion quat = 
            tf::createQuaternionMsgFromYaw(yaw_displacement);
        
        // Publish the Transform odom->base_link
        if (this->broadcast_tf) {
            this->odom_trans.header.stamp = current_time;
            
            this->odom_trans.transform.translation.x = this->odometry_x;
            this->odom_trans.transform.translation.y = this->odometry_y;
            this->odom_trans.transform.translation.z = 0.0;
            this->odom_trans.transform.rotation = quat;
            
            //send the transform
            this->odom_broadcaster.sendTransform(this->odom_trans);
        }
        
        // Publish Odometry
        this->odom_msg.header.stamp = current_time;
        this->odom_msg.pose.pose.position.x = this->odometry_x;
        this->odom_msg.pose.pose.position.y = this->odometry_y;
        this->odom_msg.pose.pose.position.z = 0.0;
        this->odom_msg.pose.pose.orientation = quat;
        this->odom_msg.pose.covariance[0] = 0.00001;
        this->odom_msg.pose.covariance[7] = 0.00001;
        this->odom_msg.pose.covariance[14] = 1000000000000.0;
        this->odom_msg.pose.covariance[21] = 1000000000000.0;
        this->odom_msg.pose.covariance[28] = 1000000000000.0;
        this->odom_msg.pose.covariance[35] = 0.001;
        
        this->odom_msg.twist.twist.linear.x = vel_x;
        this->odom_msg.twist.twist.linear.y = vel_y;
        this->odom_msg.twist.twist.angular.z = yaw_rate;
        
        this->odom_pub.publish(this->odom_msg);
    }
    
    void motor_timeoutCallback(const ros::TimerEvent& e) {
        boost::mutex::scoped_lock lock(m_mutex);
        this->target_linear_vel = 0.0;
        this->angular_vel = 0.0;
    }
    
    void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        boost::mutex::scoped_lock lock(m_mutex);
        double x = msg->linear.x, z = msg->angular.z;
        if (this->invert_x) {
            x *= -1;
        }
        if (this->invert_z) {
            z *= -1;
        }
        this->linear_vel = x;
        this->angular_vel = z;
        this->motor_timeout_timer = 
            this->n->createTimer(
                ros::Duration(this->segway_motor_timeout),
                &SegwayRMPNode::motor_timeoutCallback, 
                this, 
                true);
    }
private:
    // Functions
    void setupROSComms() {
        // Subscribe to command velocities
        this->cmd_velSubscriber = n->subscribe("cmd_vel", 1000, &SegwayRMPNode::cmd_velCallback, this);
        // Advertise the SegwayStatusStamped
        this->segway_status_pub = n->advertise<segway_rmp200::SegwayStatusStamped>("segway_status", 1000);
        // Advertise the Odometry Msg
        this->odom_pub = n->advertise<nav_msgs::Odometry>("odom", 50);
    }
    
    void setupSegwayRMP() {
        std::stringstream ss;
        ss << "Connecting to Segway RMP via ";
        this->segway_rmp = new segwayrmp::SegwayRMP(this->interface_type);
        if (this->interface_type_str == "serial") {
            ss << "serial on serial port: " << this->serial_port;
            this->segway_rmp->configureSerial(this->serial_port);
        } else if (this->interface_type_str == "usb") {
            ss << "usb ";
            if (this->usb_selector == "serial_number") {
                ss << "identified by the device serial number: " << this->serial_number;
                this->segway_rmp->configureUSBBySerial(this->serial_number);
            }
            if (this->usb_selector == "description") {
                ss << "identified by the device description: " << this->usb_description;
                this->segway_rmp->configureUSBByDescription(this->usb_description);
            }
            if (this->usb_selector == "index") {
                ss << "identified by the device index: " << this->usb_index;
                this->segway_rmp->configureUSBByIndex(this->usb_index);
            }
        }
        ROS_INFO("%s", ss.str().c_str());
        
        // Set the instance variable
        segwayrmp_node_instance = this;
        
        // Set callbacks for segway data and messages
        this->segway_rmp->setStatusCallback(handleStatusWrapper);
        this->segway_rmp->setDebugMsgCallback(handleDebugMessages);
        this->segway_rmp->setInfoMsgCallback(handleInfoMessages);
        this->segway_rmp->setErrorMsgCallback(handleErrorMessages);
    }
    
    int getParameters() {
        // Get Interface Type
        n->param("interface_type", this->interface_type_str, std::string("serial"));
        // Get Configurations based on Interface Type
        if (this->interface_type_str == "serial") {
            this->interface_type = segwayrmp::serial;
            n->param("serial_port", this->serial_port, std::string("/dev/ttyUSB0"));
        } else if (this->interface_type_str == "usb") {
            this->interface_type = segwayrmp::usb;
            n->param("usb_selector", this->usb_selector, std::string("index"));
            if (this->usb_selector == "index") {
                n->param("usb_index", this->usb_index, 0);
            } else if (this->usb_selector == "serial_number") {
                n->param("usb_serial_number", this->serial_number, std::string("00000000"));
                if (this->serial_number == std::string("00000000")) {
                    ROS_WARN("The serial_number parameter is set to the default 00000000, which shouldn't work.");
                }
            } else if (this->usb_selector == "description") {
                n->param("usb_description", this->serial_number, std::string("Robotic Mobile Platform"));
            } else {
                ROS_ERROR(
                    "Invalid USB selector: %s, valid types are 'index', 'serial_number', and 'description'.", 
                    this->usb_selector.c_str());
                return 1;
            }
        } else {
            ROS_ERROR(
             "Invalid interface type: %s, valid interface types are 'serial' and 'usb'.",
             this->interface_type_str.c_str());
            return 1;
        }
        // Get Setup Motor Timeout
        n->param("segway_motor_timeout", this->segway_motor_timeout, 0.5);
        // Get frame id parameter
        n->param("frame_id", frame_id, std::string("base_link"));
        this->sss_msg.header.frame_id = this->frame_id;
        this->odom_trans.header.frame_id = "odom";
        this->odom_trans.child_frame_id = this->frame_id;
        this->odom_msg.header.frame_id = "odom";
        this->odom_msg.child_frame_id = this->frame_id;
        // Get cmd_vel inversion parameters
        n->param("invert_x", invert_x, false);
        n->param("invert_z", invert_z, false);
        // Get option for enable/disable tf broadcasting
        n->param("broadcast_tf", this->broadcast_tf, true);
        return 0;
    }
    
    // Variables
    ros::NodeHandle * n;
    
    ros::Timer keep_alive_timer;
    
    ros::Subscriber cmd_velSubscriber;
    ros::Publisher segway_status_pub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
    
    segwayrmp::SegwayRMP * segway_rmp;
    
    std::string interface_type_str;
    segwayrmp::InterfaceType interface_type;
    std::string serial_port;
    std::string usb_selector;
    std::string serial_number;
    std::string usb_description;
    int usb_index;
    
    double segway_motor_timeout;
    ros::Timer motor_timeout_timer;
    
    std::string frame_id;
    bool invert_x, invert_z;
    bool broadcast_tf;
    
    double target_linear_vel;
    double linear_vel;
    double angular_vel;
    
    bool connected;
    
    segway_rmp200::SegwayStatusStamped sss_msg;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom_msg;
    
    int count;
    
    bool first_odometry;
    float last_forward_displacement;
    float last_yaw_displacement;
    float odometry_x;
    float odometry_y;
    float odometry_w;
    ros::Time last_time;
    
    boost::mutex m_mutex;
};

// Callback wrapper
void handleStatusWrapper(segwayrmp::SegwayStatus &ss) {
    segwayrmp_node_instance->handleStatus(ss);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "segway_rmp200");
    
    SegwayRMPNode segwayrmp_node;
    
    segwayrmp_node.run();
    
    return 0;
}