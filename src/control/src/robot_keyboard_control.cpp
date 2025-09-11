#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Callback function for robot position
nav_msgs::Odometry robotPosition;
void doRobotPos(const nav_msgs::Odometry::ConstPtr& msg_p)
{
    robotPosition = *msg_p;
}

// Callback function for receiving keyboard input
std_msgs::Int16 keyboard_msgs;
void do_Keyboard(const std_msgs::Int16::ConstPtr& msg_keyboard)
{
    keyboard_msgs = *msg_keyboard;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "robot_keyboard_control");
    ros::NodeHandle nh;
    
    // Subscriber for keyboard input
    ros::Subscriber keyboard_sub = nh.subscribe<std_msgs::Int16>("keyboard_msg", 10, do_Keyboard);
    // Publisher for velocity commands
    ros::Publisher robot_vel_cmd_pub = nh.advertise<geometry_msgs::Twist>("/laser_robot/cmd_vel", 10);
    // Subscriber for robot odometry
    ros::Subscriber robot_pos_sub = nh.subscribe<nav_msgs::Odometry>("/laser_robot/odom", 10, doRobotPos);

    ros::Rate rate(30);
    // Variable for robot velocity
    geometry_msgs::Twist robot_vel;
    
    while (ros::ok())
    {
        switch (keyboard_msgs.data)
        {
        // Move forward
        case 8:
            robot_vel.linear.x = 1.0;
            break;
        // Move backward
        case 2:
            robot_vel.linear.x = -1.0;
            break;
        // Turn left
        case 4:
            robot_vel.angular.z = 1.04712;
            break;
        // Turn right
        case 6:
            robot_vel.angular.z = -1.04712;
            break;
        // Stop
        case 5:
            robot_vel.linear.x = 0;
            robot_vel.angular.z = 0;
            break;
        default:
            break;
        }
        // Publish robot velocity
        robot_vel_cmd_pub.publish(robot_vel);
        ros::spinOnce();
        rate.sleep();
    }
}