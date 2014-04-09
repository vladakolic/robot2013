#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "ros/ros.h"
#undef INT_MAX

#include <global_position_estimation/Options.h>
#include <ir_obstacle_detection/IRObstacleSignal.h>
#include <odometry_data/Odometry.h>


ros::Subscriber g_EncoderOdometrySub;
ros::Subscriber g_IRObstacleSignalSub;
ros::Subscriber g_OptionsSub;

ros::Publisher g_OdometryPub;

float g_fRobotX;
float g_fRobotY;
float g_fRobotPhi;
float g_fVLin;
float g_fVRot;

float g_fLeftWallTheta;
float g_fRightWallTheta;
float g_fLeftHypoLength;
float g_fRightHypoLength;

float g_fOldThetaEstimate;
float g_fNewThetaEstimate;

global_position_estimation::Options::ConstPtr g_pOptions;

void receiveOptions(const global_position_estimation::Options::ConstPtr pMsg)
{
    //if (pMsg->bActivateThetaImprovement)
        //ROS_INFO("theta improvement is activated");
    //else
        //ROS_INFO("theta improvement is deactivated");

    g_pOptions = pMsg;
}

void receiveEncoderOdometry(const odometry_data::Odometry::ConstPtr & pMsg)
{
    g_fRobotX = pMsg->x;
    g_fRobotY = pMsg->y;
    g_fRobotPhi = pMsg->theta;
    g_fVLin = pMsg->V_lin;
    g_fVRot = pMsg->V_rot;
}

void receiveIRObstacleSignal(const ir_obstacle_detection::IRObstacleSignal::ConstPtr & pMsg)
{
    g_fLeftHypoLength = pMsg->left_hypo_length;
    g_fRightHypoLength = pMsg->right_hypo_length;
    g_fLeftWallTheta = pMsg->left_theta;
    g_fRightWallTheta = pMsg->right_theta;
}

void reestimateTheta(){
    // phi is the global absolute angle relative to initial robot coordinate system.
    // theta is the relative angle between the robot and the wall. BI(a)TCH.
    float fWallTheta;
    if (g_fLeftHypoLength > 0 && g_fRightHypoLength > 0){
        fWallTheta = 0.5 * (g_fLeftWallTheta + g_fRightWallTheta);
    } else {
        fWallTheta = g_fLeftWallTheta + g_fRightWallTheta;
    }
    //fWallTheta = (float) round(fWallTheta);
    float fRelativeToWallEncoderValue = g_fRobotPhi > 0 ? fmod(g_fRobotPhi + 0.25*M_PI, 0.5*M_PI) - 0.25*M_PI : fmod(g_fRobotPhi - 0.25*M_PI, 0.5*M_PI) + 0.25*M_PI;
    
    float k = 0.990;
    /*
     * Only improve theta if the option is set.
     */
    /*
    if (g_pOptions)
    {
        if (g_pOptions->bActivateThetaImprovement)
            k = 0.995;
        else
            k = 1.0;
    }
    else
    {
        k = 0.90;
    }
    */

    float fWeightedMeanTheta = (1-k) * fWallTheta + k * fRelativeToWallEncoderValue;
    float g_fNewPhiEstimate = g_fRobotPhi - (fRelativeToWallEncoderValue - fWeightedMeanTheta);

    if (g_fLeftHypoLength < 0.001 && g_fRightHypoLength < 0.001){
        g_fNewPhiEstimate = g_fRobotPhi;
    }
    
    //ROS_INFO("Estimated theta: %f", 180*g_fNewPhiEstimate/M_PI);
    //ROS_INFO("Encoder theta:   %f", 180*g_fRobotPhi/M_PI);
    //ROS_INFO("Encoder phi:     %f", 180*fRelativeToWallEncoderValue/M_PI);
    //ROS_INFO("Left Wall phi:   %f", (float) round(180*g_fLeftWallTheta/M_PI));
    //ROS_INFO("Right Wall phi:  %f", 180*g_fRightWallTheta/M_PI);


    odometry_data::Odometry toPub;
    toPub.x = g_fRobotX;
    toPub.y = g_fRobotY;
    toPub.theta = g_fNewPhiEstimate;
    toPub.V_lin = g_fVLin;
    toPub.V_rot = g_fVRot;

    g_OdometryPub.publish(toPub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_position_estimation");

    ros::NodeHandle n;

    // Connect to mapping node
    //g_ObstaclePub = n.advertise<map_creation::ObstacleLine>("/ir_obstacle_detection/ObstacleLine", 10);
    //g_ObstaclePub = n.advertise<ir_obstacle_detection::IRObstacleSignal>("/ir_obstacle_detection/IRObstacleSignal",10);

    // Connect to arduino for ir sensor data
    g_EncoderOdometrySub = n.subscribe("/motion/EncoderOdometry", 1, receiveEncoderOdometry);
    g_IRObstacleSignalSub = n.subscribe("/ir_obstacle_detection/IRObstacleSignal", 1, receiveIRObstacleSignal);
    g_OptionsSub = n.subscribe("/global_position_estimation/Options", 1, receiveOptions);

    g_OdometryPub = n.advertise<odometry_data::Odometry>("/motion/Odometry",1);

    // TODO publish... if you wish... and make a dish ... with a fish ...
    // Odometry must suscribe to new estimate of PHI and test this shiet.

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        // detect obstacles
        //detectObstacles();
        reestimateTheta();
        
        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
