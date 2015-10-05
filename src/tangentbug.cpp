
/*

This file containts the functions for publishing a constant setpoint
for a differential robot

*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>

#define _x_   0
#define _y_   1
#define _a_   2

#define PI    3.14159265359
#define INF   100000.0

// Helper
double abs(double a) { return a >= 0 ? a : -a; }

double sign(double a) { return a >= 0 ? 1.0 : (a < 0.0 ? -1.0 : 0.0); }

double rad2deg(double a) { return 180.0/PI * a; }

// Controller

// TODO: improve collision avoidance

class Controller
{
    enum class Action { Stop,
                        StopAtGoal,
                        TurnAtGoal,
                        GoGoal,
                        GoTangent,
                        TurnRight };

public:
    // constructor
    Controller();

    // operation
    void UpParam(char** argv);
    void Subscribe(ros::NodeHandle& nh);
    void UpVel(void);

    double getAngVel(void) { return va; }
    double getLinVel(void) { return vl; }

private:
    // callbacks
    void UpSetpoint(const geometry_msgs::Pose::ConstPtr& setpoint);
    void UpPose(const nav_msgs::Odometry::ConstPtr& pose);
    void UpScan(const sensor_msgs::LaserScan::ConstPtr& scan);

    // auxiliary
    bool CheckGoalDir(double goal_dir, double goal_dist);

    // variables
    double po[3];   // current pose
    double sp[3];   // setpoint

    double vl, va;  // desired velocities

    double vdmax, vamax, atoli, atole, dtoli, dtole;    // parameters

    constexpr static double wid = 0.5;     // a bit more then half the robot's width

    // laser scan info
    constexpr static size_t samples = 180;
    double bmin, bmax, binc, rmin, rmax;
    double ranges[samples], angles[samples];

    // state variables
    int wasTurningRight;
    int wasFollowingTangent;
    int wasAtGoal;

    // ros msg subscribers
    ros::Subscriber sp_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber scan_sub;
};

Controller::Controller()
{
    // current pose
    po[_x_] = 0.0;
    po[_y_] = 0.0;
    po[_a_] = 0.0;

    // setpoint
    sp[_x_] = 0.0;
    sp[_y_] = 0.0;
    sp[_a_] = 0.0;

    // desired velocity
    vl = 0.0;
    va = 0.0;

    // controller parameters
    vdmax = 2.0;
    vamax = 1.0;
    atoli = 0.1;
    atole = 0.5;
    dtoli = 0.1;
    dtole = 0.5;

    // laser scan info
    bmin = -PI/2;
    bmax = PI/2;
    binc = (bmax - bmin)/(samples-1);
    rmin = 0.0;
    rmax = 9.5;
    for(size_t i = 0; i < samples; i++)
    {
        ranges[i] = 0.0;
        angles[i] = bmin + i*binc;
    }

    // state variables
    wasAtGoal = 0;
    wasFollowingTangent = 0;
    wasTurningRight = 0;
}

void Controller::UpParam(char** argv)
{
    vdmax = atof(argv[0]);
    vamax = atof(argv[1]);
    dtoli = atof(argv[2]);
    dtole = atof(argv[3]);
    atoli = atof(argv[4]);
    atole = atof(argv[5]);
}

void Controller::Subscribe(ros::NodeHandle& nh )
{
    sp_sub = nh.subscribe<geometry_msgs::Pose>("/setpoint", 1, &Controller::UpSetpoint, this);
    pose_sub = nh.subscribe<nav_msgs::Odometry>("/pose", 1, &Controller::UpPose, this);
    scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Controller::UpScan, this);
}

void Controller::UpSetpoint(const geometry_msgs::Pose::ConstPtr& setpoint)
{

    sp[_x_] = setpoint->position.x;
    sp[_y_] = setpoint->position.y;
    sp[_a_] = tf::getYaw(setpoint->orientation);
}

void Controller::UpPose(const nav_msgs::Odometry::ConstPtr& pose)
{
    po[_x_] = pose->pose.pose.position.x;
    po[_y_] = pose->pose.pose.position.y;
    po[_a_] = tf::getYaw(pose->pose.pose.orientation);
}

void Controller::UpScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(size_t i = 0; i < samples; i++)
    {
        ranges[i] = scan->ranges[i];
    }
}

bool Controller::CheckGoalDir(double goal_dir, double goal_dist)
{
    // checks if there's an obstacle in the direction of the goal
    // considering the robot's width

    for(size_t i = 0; i < samples; i++)
    {
        if( goal_dir > 0.0 && bmin+goal_dir > angles[i] ) continue;

        if( goal_dir < 0.0 && bmax+goal_dir < angles[i] ) continue;

        if( goal_dist > ranges[i] &&
            abs(ranges[i]*sin(angles[i] - goal_dir)) <= wid )
            return false;
    }

    return true;
}

void Controller::UpVel(void)
{
    bool isGoalBehind = false;
    bool isGoalPathFree = false;
    bool isAnyTanSel = false;
    bool shouldTryTan = false;

    size_t numTan = 0;

    Action action = Action::Stop;

    // delta 'dl' and 'da' to goal
    double dx = sp[_x_] - po[_x_];
    double dy = sp[_y_] - po[_y_];
    double dl = hypot(dx,dy);
    double da = atan2(dy,dx) - po[_a_];

    ROS_INFO("--------");
    ROS_INFO("  to goal: dx: %.3f, dy: %.3f, da: %.3f, dl: %.3f", dx, dy, rad2deg(da), dl);

    // choose action
    if( dl < dtoli || (dl < dtole && wasAtGoal) )
    {
        // at goal
        if( abs(da) < atoli )
            action = Action::StopAtGoal;
        else
            action = Action::TurnAtGoal;
    }
    else if( da >= bmax || da <= bmin )
    {
        // goal is behind robot
        isGoalBehind = true;

        if( wasFollowingTangent || wasTurningRight )
            shouldTryTan = true;
        else
            action = Action::GoGoal;
    }
    else if( CheckGoalDir(da, dl) )
    {
        // goal is in front of robot
        isGoalPathFree = true;
        action = Action::GoGoal;
    }
    else
        shouldTryTan = true;

    // try to follow a tangent
    if( shouldTryTan )
    {
        // list free tangents
        // angles, ranges, and corrections
        double angl[samples], rang[samples], corr[samples];

        for(size_t i = 0; i < samples-1; i++)
        {
            double delta = ranges[i] - ranges[i+1];

            if( delta > 2*wid || (ranges[i] > rmax && ranges[i+1] < rmax) )
            {
                // 'closing' discontinuity
                angl[numTan] = angles[i];
                rang[numTan] = ranges[i+1];
                corr[numTan] = -asin(wid/ranges[i+1]);
                numTan++;
            }
            else if( delta < -2*wid || (ranges[i] < rmax && ranges[i+1] > rmax) )
            {
                // 'opening' discontinuity
                angl[numTan] = angles[i+1];
                rang[numTan] = ranges[i];
                corr[numTan] = asin(wid/ranges[i]);
                numTan++;
            }
        }

        // select best tangent
        double an, ra, co;

        if( numTan > 0 )
        {
            an = INF;
            ra = 0.0;
            co = 0.0;
            // selects tangent which is in the nearest direction
            for(size_t i = 0; i < numTan; i++)
            {
                ROS_INFO("    tan: angl: %.3f, corr: %.3f, rang: %.3f", rad2deg(angl[i]), rad2deg(corr[i]), rang[i]);

                if( abs(angl[i] - da) < abs(an - da) )
                {
                    an = angl[i];
                    ra = rang[i];
                    co = corr[i];
                    isAnyTanSel = true;
                }
            }
            // TODO FIXME sometimes the algorithm enters an endless loop
            // ie., the choice of tangent oscilates between two options forever
            //if( numTan > 1 )
            //{
                //for(size_t i = 0; i < numTan; i++)
                //{
                    //if( co > 0.0 && (angl[i] < an+2*co && angl[i] > an) && ra > rang[i] )
                    //{
                        //an = angl[i];
                        //ra = rang[i];
                        //co = corr[i];
                    //}
                    //else if( co < 0.0 && (angl[i] > an+2*co && angl[i] < an) && ra > rang[i] )
                    //{
                        //an = angl[i];
                        //ra = rang[i];
                        //co = corr[i];
                    //}
                //}
            //}
        }

        if( isAnyTanSel )
        {
            // some tangent selected
            ROS_INFO("     sel: angl: %.3f, corr: %.3f, rang: %.3f", rad2deg(an), rad2deg(co), ra);

            dl = ra;
            da = an + co;
            action = Action::GoTangent;
        }
        else
            // no tangent selected
            action = Action::TurnRight;
    }

    ROS_INFO("  states: iGB: %d, iGPF: %d, sTT: %d, aT: %d", isGoalBehind, isGoalPathFree, shouldTryTan, isAnyTanSel);
    ROS_INFO("    past: wTR: %d, wFT: %d, wAG: %d", wasTurningRight, wasFollowingTangent, wasAtGoal);

    // update state
    wasAtGoal = false;
    wasFollowingTangent = false;
    wasTurningRight = false;

    switch( action )
    {
        case Action::StopAtGoal:
            wasAtGoal = true;
            dl = 0.0;
            da = 0.0;
            ROS_INFO("  StopAtGoal");
            break;
        case Action::TurnAtGoal:
            wasAtGoal = true;
            dl = 0.0;
            da = sp[_a_] - po[_a_];
            ROS_INFO("  TurnAtGoal");
            break;
        case Action::GoGoal:
            ROS_INFO("  GoGoal");
            break;
        case Action::GoTangent:
            wasFollowingTangent = true;
            ROS_INFO("  GoTangent");
            break;
        case Action::TurnRight:
            wasTurningRight = true;
            dl = 0.0;
            da = -0.4;
            ROS_INFO("  TurnRight");
            break;
        case Action::Stop:
        default:
            dl = 0.0;
            da = 0.0;
            ROS_INFO("  Stop");
            break;
    }

    // fuzzy controller
    double k;
    if( wasAtGoal )
        k = 0.0;
    else if( dl >= dtole )
        k = 1.0;
    else
        k = (dtole - dl)/(dtole - dtoli);

    if( da >= atole ) {
        vl = 0.0;
        va = vamax;
    } else if( da <= -atole ) {
        vl = 0.0;
        va = -vamax;
    } else if( da > atoli && da < atole ) {
        vl = (atole - da)/(atole - atoli) * vdmax * k;
        va = (da - atoli)/(atole - atoli) * vamax;
    } else if( da < -atoli && da > -atole ) {
        vl = (atole + da)/(atole - atoli) * vdmax * k;
        va = (da + atoli)/(atole - atoli) * vamax;
    } else {
        vl = vdmax * k;
        va = 0.0;
    }

    ROS_INFO("  dl: %.3f, da: %.3f", dl, rad2deg(da));
    ROS_INFO("  vl: %.3f, va: %.3f", vl, rad2deg(va));
}

// Main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tangentbug");

    Controller controller;

    if( argc == 7 )
        controller.UpParam( &argv[1] );

    ros::NodeHandle nh;

    controller.Subscribe( nh );

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Duration(5.0).sleep();

    ros::Rate rate(10);

    while( ros::ok() )
    {
        ros::spinOnce();

        controller.UpVel();

        geometry_msgs::Twist vel;
        vel.angular.z = controller.getAngVel();
        vel.linear.x = controller.getLinVel();
        vel_pub.publish(vel);

        rate.sleep();
    }

    return 0;
}

