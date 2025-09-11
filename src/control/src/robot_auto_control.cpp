#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>
#include <tf/tf.h>
#include <math.h>

#define PI acos(-1)

using namespace std;

double max_linear_vel = 0.8;
double min_linear_vel = 0.3;
double max_angular_vel = 3.0;

double map_ratio = 5.0;      // Initial default value
double image_size = 200.0;   // Default image size

// Convert quaternion to Euler angles
float robot_yaw;
void orientation2Eular(const nav_msgs::Odometry &odom) 
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  robot_yaw = yaw;
}

// Calculate angular velocity
double calTurn_vel(double current_yaw, double target_yaw)
{
    double v;
    if(current_yaw - target_yaw > PI || current_yaw - target_yaw < -PI)
    {
        if(current_yaw > 0)
        {
            v = 2 * PI - current_yaw + target_yaw; 
        }
        else
        {
            v = -(2 * PI + current_yaw - target_yaw);
        } 
    }
    else 
    {
        v = target_yaw - current_yaw;
    }
    if(abs(v) > max_angular_vel)
    {
        if(v > 0)
        {
            v = max_angular_vel;
        }
        else 
        {
            v = -max_angular_vel;
        }
    }
    return v;
}

// Calculate linear velocity
double calLinear_vel(double res)
{
    double vel;
    vel = res / 15;
    if(vel > max_linear_vel)
    {
        vel = max_linear_vel;
    }
    if(vel < min_linear_vel)
    {
        vel = min_linear_vel;
    }
    return vel;
}

// Callback function for robot position
nav_msgs::Odometry robotPosition;
void doRobotPos(const nav_msgs::Odometry::ConstPtr& msg_p)
{
    robotPosition = *msg_p;
    orientation2Eular(robotPosition);
}

// Callback function for path sequence
vector<vector<int>> route_list;
std_msgs::String route_list_s_last;
bool new_route_list_get = false;
void doRouteList(const std_msgs::String::ConstPtr& msg_r)
{
    std_msgs::String route_list_s = *msg_r;
    if(route_list_s_last.data != route_list_s.data && route_list_s.data.size() > 0)
    {
        route_list_s_last = route_list_s;
        route_list_s.data.erase(0, 1);
        route_list_s.data.erase(route_list_s.data.size() - 1);
        int loc0, loc1, loc2, x, y;
        vector<vector<int>>().swap(route_list);
        for(int i = 0; i < route_list_s.data.size();)
        {
            loc0 = route_list_s.data.find("[", i);
            loc1 = route_list_s.data.find(",", i);
            loc2 = route_list_s.data.find("]", i);
            x = atoi(route_list_s.data.substr(loc0 + 1, loc1 - loc0 - 1).c_str());
            y = atoi(route_list_s.data.substr(loc1 + 2, loc2 - loc1 - 2).c_str());
            route_list.push_back({x, y});
            i = loc2 + 2;
        }
        new_route_list_get = true;
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "robot_keyboard_control");
    ros::NodeHandle nh;
    
    // Get the image_size parameter
    nh.param<double>("image_size", image_size, 200.0);
    
    // Calculate map_ratio based on image_size
    // 200 corresponds to 10, 50 corresponds to 2.5, linear relation: map_ratio = image_size / 20
    map_ratio = image_size / 20.0;
    
    ROS_INFO("image_size: %.1f, map_ratio: %.2f", image_size, map_ratio);
    
    // Publisher for velocity commands
    ros::Publisher robot_vel_cmd_pub = nh.advertise<geometry_msgs::Twist>("/laser_robot/cmd_vel", 10);
    // Subscriber for robot odometry
    ros::Subscriber robot_pos_sub = nh.subscribe<nav_msgs::Odometry>("/laser_robot/odom", 10, doRobotPos);
    // Subscriber for route list
    ros::Subscriber route_list_sub = nh.subscribe<std_msgs::String>("/route_list", 10, doRouteList);

    ros::Rate rate(60);
    geometry_msgs::Twist robot_vel;
    int move_mark;
    bool done_move = false;
    
    while (ros::ok())
    {
        if(route_list.size() > 0 && (!done_move || new_route_list_get))
        {
            if(new_route_list_get)
            {
                move_mark = 0;
                new_route_list_get = false;
                done_move = false;
            }
            else
            {
                float target_x = route_list[move_mark][0];
                float target_y = route_list[move_mark][1];
                float res = pow(
                    pow(target_x - robotPosition.pose.pose.position.x * map_ratio, 2) +  
                    pow(target_y - robotPosition.pose.pose.position.y * map_ratio, 2),
                    0.5);
                robot_vel.linear.x = calLinear_vel(res);
                robot_vel.angular.z = calTurn_vel(
                    robot_yaw, 
                    atan2(target_y - robotPosition.pose.pose.position.y * map_ratio, 
                        target_x - robotPosition.pose.pose.position.x * map_ratio)
                );
                
                if(move_mark < route_list.size() - 1)
                {
                    if(abs(target_x - robotPosition.pose.pose.position.x * map_ratio) < 3.0 &&
                    abs(target_y - robotPosition.pose.pose.position.y * map_ratio) < 3.0)
                    {
                        move_mark ++;
                    }
                }
                else
                {
                    if(abs(target_x - robotPosition.pose.pose.position.x * map_ratio) < 1.0 &&
                    abs(target_y - robotPosition.pose.pose.position.y * map_ratio) < 1.0)
                    {
                        done_move = true;
                        robot_vel.linear.x = 0;
                        robot_vel.angular.z = 0;
                    }
                }
            }
            robot_vel_cmd_pub.publish(robot_vel);
        }
        ros::spinOnce();
        rate.sleep();
    }
}