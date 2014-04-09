#include <stdlib.h>

#include "ros/ros.h"

#include <hand_detection/HandDetection.h>
#include <motor_control/Speed.h>

/** Subscriber for hand detection messages. */
ros::Subscriber g_HandDetectionSub;

/** Publisher for speed messages. */
ros::Publisher g_SpeedPub;


/**
 * Translate hand position to speed instructions.
 */
void receiveHandDetection(const hand_detection::HandDetection::ConstPtr& pMsg)
{
    printf("x: %g, y: %g\n", pMsg->fX, pMsg->fY);
    printf("depth: %g \n", pMsg->fDepth);
    float xDir = pMsg->fX;
    motor_control::Speed speedMsg;
        
    if(pMsg->bIsDetected){


        if(xDir > -0.25 && xDir < 0.25) xDir = 0;
        if(xDir == 0){
            speedMsg.fSpeedLeft = 0.5;
            speedMsg.fSpeedRight = 0.5;
        } else if(xDir < 0){
            speedMsg.fSpeedLeft = -0.3;
            speedMsg.fSpeedRight = 0.3;        
        } else {
            speedMsg.fSpeedLeft = 0.3;
            speedMsg.fSpeedRight = -0.3;
        }
    }
    // speedMsg.fSpeedLeft = pMsg->bIsDetected ? (1* 0.75 + xDir) : 0.0;
    // speedMsg.fSpeedRight = pMsg->bIsDetected ? (1 * 0.75 - xDir) : 0.0;
    speedMsg.timestamp = ros::Time::now();
    g_SpeedPub.publish(speedMsg);    
}


/**
 * This node follows a hand in front of the camera and publishes control speeds to the motor_control node.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hand_following");

    ros::NodeHandle n;

    g_HandDetectionSub = n.subscribe("/human/hand_detection", 100, receiveHandDetection);
    g_SpeedPub = n.advertise<motor_control::Speed>("/motor_control/Speed", 1000);

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
