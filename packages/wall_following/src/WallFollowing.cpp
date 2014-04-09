#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdlib>

#include "ros/ros.h"
#include <ir_obstacle_detection/IRObstacleSignal.h>
#include <differential_drive/PWM.h>
#include <keyboard_control/KeyboardStates.h>

ros::Subscriber g_WallDetection;
ros::Subscriber g_KeyboardSub;
ros::Publisher g_pwmPub;

// information received from IR sensors
bool g_CanMoveForward;
bool g_CanMoveBack;
bool g_CanTurnLeft;
bool g_CanPartlyTurnLeft;
bool g_CanTurnRight;
bool g_CanPartlyTurnRight;

int  g_iTurningDegrees;
bool g_bCheckTurningDirection;

void receiveKeyboardStates(const keyboard_control::KeyboardStates::ConstPtr& pMsg)
{
    return;
    if (pMsg->bUpArrow)
        g_iTurningDegrees = 0;
    if (pMsg->bLeftArrow)
        g_iTurningDegrees = 90;
    if (pMsg->bRightArrow)
        g_iTurningDegrees = -90;
    if (pMsg->bDownArrow)
        g_iTurningDegrees = 180;
    if (pMsg->bUpArrow && pMsg->bDownArrow)
        g_iTurningDegrees = 450;
}

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
    g_CanTurnLeft 	 = !pMsg->front_left && !pMsg->rear_left;
    g_CanPartlyTurnLeft = pMsg->front_left && !pMsg->rear_left;
    g_CanTurnRight   = !pMsg->front_right && !pMsg->rear_right;
    g_CanPartlyTurnRight = pMsg->front_right && !pMsg->rear_right;
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
	if (!g_CanMoveForward){
    	// WALL (or something else) IS IN THE FRONT OF THE ROBOT, STOP IT ASAP
    	if (!g_CanTurnLeft && !g_CanTurnRight){
            if (g_CanPartlyTurnLeft || g_CanPartlyTurnRight)
                if (g_CanPartlyTurnLeft)
                    return 90;
                else if (g_CanPartlyTurnRight)
                    return -90;
			// Robot has to turn around 180 degrees.
            else if (g_CanMoveBack)
				return 180;
			else
				// looks like we have no available direction here... TIME TO PANIC
				return 405;

		}
		if (g_CanTurnLeft && g_CanTurnRight) {
			// robot can turn left and right, pick one at random
			float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			return r > 0.5f ? 90 : -90;
		}
		if (g_CanTurnLeft){
			// turn the robot to the left
    		return 90;
		}
		if (g_CanTurnRight){
			// turn the robot to the right
			return -90;
		}

    } else {
        /*
        if (!g_CanPartlyTurnLeft && g_CanPartlyTurnRight)
            return -90;
        else if (!g_CanPartlyTurnRight && g_CanPartlyTurnLeft)
            return -90;
        else
        */
            return 0;
            // NOTHING IS STOPPING US ! MOVE FORWARDDDDDDDDDDDDD
    }	
}

void debugPrint(){
	if (g_iTurningDegrees == 0)
		ROS_INFO("Moving forward");
	else if (g_iTurningDegrees == 180)
		ROS_INFO("Turning around 180 degrees");
	else if (g_iTurningDegrees == 90)
		ROS_INFO("Turning left");
	else if (g_iTurningDegrees == -90)
		ROS_INFO("Turning right");
	else
		ROS_INFO("PANICCCCCCCCCCCC");
	ROS_INFO("choosed angle: %d", g_iTurningDegrees);
}

void moveRobot(){

    differential_drive::PWM pwmMsg;
    float fSnippets = 1;
    int nSpeedLeft = 90;
    int nSpeedRight = 110;

		g_iTurningDegrees = calculateMovingDirection();
        debugPrint();
        
	if (g_iTurningDegrees == 0){
		// keep moving forward
		g_bCheckTurningDirection = true;
        pwmMsg.PWM1 = nSpeedLeft;//50;
        pwmMsg.PWM2 = -nSpeedRight;//50;
	} else if (g_iTurningDegrees > 360) {
		// dont know what to do here. try turning 45 degrees maybe?
		// in this case we might be able to twist and turn our way out of this mess.
        pwmMsg.PWM1 = -nSpeedLeft;
        pwmMsg.PWM2 = -nSpeedRight;
	} else {
		// turn the robot degrees_to_turn degrees. 
		g_bCheckTurningDirection = false; // have to finish this turn before we check what direction to turn again

		// TODO: TURN THE ROBOT g_TurningDirection DEGREES HERE.
		// SET g_bCheckTurningDirection TO TRUE WHEN THIS IS DONE.
        // idea: move in 45 degree snippets
        fSnippets = abs(g_iTurningDegrees) / 45 - 0.25;
        pwmMsg.PWM1 = -nSpeedLeft;
        pwmMsg.PWM2 = -nSpeedRight;

        if (g_iTurningDegrees < 0)
        {
            pwmMsg.PWM1 = -pwmMsg.PWM1;
            pwmMsg.PWM2 = -pwmMsg.PWM2;
        }
	}

    g_pwmPub.publish(pwmMsg);

    ros::Duration(0.40 * fSnippets).sleep();

    pwmMsg.PWM1 = 0;
    pwmMsg.PWM2 = 0;
    g_pwmPub.publish(pwmMsg);
    
    ros::Duration(2.0).sleep();
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_following");

    ros::NodeHandle n;

    g_WallDetection = n.subscribe("/ir_obstacle_detection/IRObstacleSignal", 1, receiveObstacleData); // message from ir sensors
    g_KeyboardSub = n.subscribe("/keyboard_control/KeyboardStates", 1, receiveKeyboardStates);

    g_pwmPub = n.advertise<differential_drive::PWM>("/motion/PWM", 1);

    ros::Rate loop_rate(100);

    // check turning direction from the beginning
    g_bCheckTurningDirection = true;

    while (ros::ok())
    {
    	moveRobot();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
