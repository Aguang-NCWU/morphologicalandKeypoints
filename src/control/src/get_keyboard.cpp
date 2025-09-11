#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <conio.h> // Download <conio.h> from https://github.com/zoelabbb/conio.h/tree/Version, use version V1.0
#include <std_msgs/Int16.h>

using namespace std;

// cmd_vel
// 56 8 Up
// 50 2 Down
// 52 4 Left
// 54 6 Right
// 53 5 Stop

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;
    ros::Publisher key_msgs_pub = nh.advertise<std_msgs::Int16>("keyboard_msg", 10);
    std_msgs::Int16 keyboard_msg;
    int key1, key2, key3;
    ros::Rate r(30);

    while (ros::ok())
    {
        if (_kbhit())
        {   // If a key is pressed, the _kbhit() function returns true
            key1 = _getche(); // Use _getche() to get the pressed key code value
            if (key1 == 27)
            {
                key2 = _getche();
                if (key2 == 27)
                {
                    break;
                }
            }

            switch (key1)
            {
            case 56:
                keyboard_msg.data = 8;
                break;
            case 50:
                keyboard_msg.data = 2;
                break;
            case 52:
                keyboard_msg.data = 4;
                break;
            case 54:
                keyboard_msg.data = 6;
                break;
            case 53:
                keyboard_msg.data = 5;
                break;
            // case 55:
            //     keyboard_msg.data = 7;
            //     break;
            // case 57:
            //     keyboard_msg.data = 9;
            //     break;
            // case 49:
            //     keyboard_msg.data = 1;
            //     break;
            // case 51:
            //     keyboard_msg.data = 3;
            //     break;
            default:
                break;
            }
        }
        key_msgs_pub.publish(keyboard_msg);
        r.sleep();
    }

    system("pause");
    return 0;
}