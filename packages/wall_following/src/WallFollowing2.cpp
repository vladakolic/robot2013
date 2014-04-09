#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdlib>

#include "ros/ros.h"
#include <ir_obstacle_detection/IRObstacleSignal.h>

ros::Subscriber g_WallDetection;

// information received from IR sensors
float g_CanMoveForward;
float g_CanMoveBack;
float g_CanTurnLeft;
float g_CanTurnRight;

float g_fThetaLeft;
float g_fThetaRight;
float g_fLeftHypoLength;
float g_fRightHypoLength;

int  g_iTurningDegrees;
bool g_bCheckTurningDirection;

void receiveObstacleData(const ir_obstacle_detection::IRObstacleSignal::ConstPtr& pMsg){
	/*
	ROS_INFO("front: %d",  pMsg->front);
    ROS_INFO("front left : %d", pMsg->front_left);
    ROS_INFO("front right : %d",  pMsg->front_right);
    ROS_INFO("rear left : %d",  pMsg->rear_left);
    ROS_INFO("rear right : %d",  pMsg->rear_right);
    ROS_INFO("back: %d",  pMsg->back);
	*/

	// checks which way we can turn... More solid approach is turn slightly to the right if
	// pMsg->front_left is closer to the wall than pMsg->rear_left, etc. Then input message
	// should be float, not boolean.
    g_CanMoveForward = !pMsg->front;
    g_CanMoveBack    = !pMsg->back;
    g_CanTurnLeft 	 = !(pMsg->front_left || pMsg->rear_left);
    g_CanTurnRight   = !(pMsg->front_right || pMsg->rear_right);

    g_fThetaLeft = pMsg->left_theta;
    g_fThetaRight = pMsg->right_theta;
    g_fLeftHypoLength = pMsg->left_hypo_length;
    g_fRightHypoLength = pMsg->right_hypo_length;

    ROS_INFO("left hypotenuse length  = %f", g_fLeftHypoLength);
    ROS_INFO("right hypotenuse length = %f", g_fRightHypoLength);
    ROS_INFO("theta left  = %f", 360*g_fThetaLeft/(2*M_PI));
    ROS_INFO("theta right = %f", 360*g_fThetaRight/(2*M_PI));
}

/**
 * Method that decides the moving direction of the robot. This should only
 * be checked before deciding a moving direction, not during.
 * @return
 * 	the direction (in degrees) to turn, i.e.
 *		0 		-> move forward
 *		90 		-> turn left 90 degrees
 *		-90 	-> turn right 90 degrees
 *		180 	-> turn around 180 degrees
 *		number higher than 360 -> no available direction to turn
 */
int calculateMovingDirection(){
	bool left = true, right = true;
	if (g_fLeftHypoLength != 0 && g_fRightHypoLength != 0){
		float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		if(r > 0.5f)
			right = false;
		else
			left = false;
	} else if (g_fLeftHypoLength != 0 && left){
		return 180*g_fThetaLeft/M_PI;
	} else if(g_fRightHypoLength != 0 && right){
		return -180*g_fThetaRight/M_PI;
	} else {
		return 0; // move forwards
	}
}

void debugPrint(){
	int decision = calculateMovingDirection();
	ROS_INFO("moving direction: %d", decision);
	//ROS_INFO("check turning direction? %d", g_bCheckTurningDirection);
}

void moveRobot(){
	debugPrint();

	if (g_bCheckTurningDirection) {
		g_iTurningDegrees = calculateMovingDirection();
	}
	if (g_iTurningDegrees == 0){
		// keep moving forward
		g_bCheckTurningDirection = true;
	} else if (g_iTurningDegrees > 360) {
		// dont know what to do here. try turning 45 degrees maybe?
		// in this case we might be able to twist and turn our way out of this mess.
	} else {
		// turn the robot degrees_to_turn degrees. 
		g_bCheckTurningDirection = false; // have to finish this turn before we check what direction to turn again

		// TODO: TURN THE ROBOT g_TurningDirection DEGREES HERE.
		// SET g_bCheckTurningDirection TO TRUE WHEN THIS IS DONE.
	}
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_following");

    ros::NodeHandle n;

    g_WallDetection = n.subscribe("/ir_obstacle_detection/IRObstacleSignal", 100, receiveObstacleData); // message from ir sensors



    ros::Rate loop_rate(50);

    while (ros::ok())
    {
    	moveRobot();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}