
/*

This file containts the functions for publishing a constant setpoint
for a differential robot

*/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

#define _x_   0
#define _y_   1
#define _a_   2

#define PI    3.14159265359

// Helper
double rad2deg(double a) { return 180.0/PI * a; }

// Main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "setpoint");

    double sp[] = {1, 1, 0};

    if( argc == 4 )
    {
        sp[_x_] = atof( argv[1] );
        sp[_y_] = atof( argv[2] );
        sp[_a_] = PI/180.0*atof( argv[3] );
    }
    ROS_INFO("Fixed Setpoint: x: %.3f, y: %.3f, a: %.3f", sp[_x_], sp[_y_], rad2deg(sp[_a_]));

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("setpoint", 1);

    geometry_msgs::Pose pose;

    ros::Rate rate(5);

    while( ros::ok() )
    {
        pose.position.x = sp[_x_];
        pose.position.y = sp[_y_];
        pose.orientation = tf::createQuaternionMsgFromYaw(sp[_a_]);

        pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

