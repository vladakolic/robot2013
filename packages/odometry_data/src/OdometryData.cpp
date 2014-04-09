#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
 
#include "ros/ros.h"
#include <differential_drive/Speed.h>
#include <robot_utilities/RobotProperties.h>
#include <odometry_data/Odometry.h>
#include <differential_drive/Odometry.h>

 
 
ros::Subscriber g_SpeedSub;
ros::Subscriber g_OdometrySub;
ros::Publisher g_OdometryPub;
 
/** Timestamp from last odometry message. */
ros::Time g_SpeedTimestamp;
 
float g_fTheta = 0;
float g_fX = 0;
float g_fY = 0;
 
float g_fGlobalPositionTheta;
float g_fGlobalPositionX;
float g_fGlobalPositionY;
float g_fGlobalPositionVlin;
float g_fGlobalPositionVrot;
 
float calculateDeltaT(ros::Time& timestamp, float fMinDeltaT)
{
    ros::Time currentTime = ros::Time::now();
    float fDeltaT = (currentTime - timestamp).toSec();
    timestamp = currentTime;

    if (fDeltaT < fMinDeltaT)
        ROS_WARN("fDeltaT is currently less than %g!", fMinDeltaT);

    return fDeltaT;
}

/**
 * Receive odometry values from the motors.
 */
void receiveSpeed(const differential_drive::Speed::ConstPtr& pMsg)
{
 
    float m1speed = pMsg->W1;
    float m2speed = pMsg->W2;
 
    float fV_lin = 0.5*(m1speed+m2speed);
    float fV_rot = 1.0/robot_properties::g_fWheelDisplacement*(m2speed-m1speed);
 
    g_fTheta = g_fGlobalPositionTheta;

    float fDeltaT = calculateDeltaT(g_SpeedTimestamp, 1.0f/101.0f);

    /*
     * !Notice: Positive theta is in counter-clockwise direction and not limited to the interval [0,2 * PI]
     */
    g_fTheta += fV_rot*fDeltaT;
 //   if(g_fTheta<0){g_fTheta += 2*M_PI;}
 //   g_fTheta = fmod(g_fTheta, (2.0 * M_PI));

    /*
     * !Notice: The global world frame is defined like this:
     *
     *  +Y
     *
     *  ^
     *  |
     *  |
     *  o-----> +X
     *
     */
    g_fX += fV_lin*fDeltaT*cos(g_fTheta);
    g_fY += fV_lin*fDeltaT*sin(g_fTheta);

    

    odometry_data::Odometry toPub;
    toPub.y = g_fX; // FORWARD AT ORIGO SHOULD CHANGE Y NOT FUCKING X.
    toPub.x = -g_fY; // I DONT HAVE TO EXPLAIN THIS AGAIN: SEE ABOVE.
    toPub.theta = g_fTheta;// + 0.8*g_fGlobalPositionTheta;
    toPub.V_lin = fV_lin;
    toPub.V_rot = fV_rot;

    g_OdometryPub.publish(toPub);
/*
    ROS_INFO("W1 = %f", m1speed);
    ROS_INFO("W2 = %f", m2speed);
    ROS_INFO("V_rot = %f", fV_rot);
    ROS_INFO("V_lin = %f", fV_lin);

    ROS_INFO("theta = %f", 180*g_fTheta/3.1415f);
    ROS_INFO("x = %f", g_fX);
    ROS_INFO("y = %f", g_fY);
    */
}

void receiveOdometry(const odometry_data::Odometry::ConstPtr& pMsg){
    g_fGlobalPositionTheta = pMsg->theta;
    g_fGlobalPositionX = pMsg->x;
    g_fGlobalPositionY = pMsg->y;
    g_fGlobalPositionVlin = pMsg->V_lin;
    g_fGlobalPositionVrot = pMsg->V_rot;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "OdometryData");
 
    

    ros::NodeHandle n;
    g_SpeedTimestamp = ros::Time::now();
 
    //g_OdometryTimestamp = ros::Time::now();
 
    // Initialize subscriber
    g_SpeedSub = n.subscribe("/motion/Speed", 1, receiveSpeed);
    g_OdometrySub = n.subscribe("/motion/Odometry", 1, receiveOdometry);
    g_OdometryPub = n.advertise<odometry_data::Odometry>("/motion/EncoderOdometry", 1);

 
    ros::Rate loop_rate(100);
 
    while (ros::ok())
    {
        // run the speed regulation
        //sendOdometryData();
 
        loop_rate.sleep();
        ros::spinOnce();
    }
 
    return EXIT_SUCCESS;
}
