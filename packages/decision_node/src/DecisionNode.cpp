#define _USE_MATH_DEFINES
#define STATE_l (g_pIRData->front_left || g_pIRData->rear_left) //leftWall

#define STATE_r (g_pIRData->front_right || g_pIRData->rear_right) //rightWall
#define STATE_f (g_pIRData->front_distance < 0.18) //frontWall
#define STATE_b (g_pIRData->back) //backWall

#define STATE_LBR (STATE_r && STATE_b && STATE_l && !STATE_f) //backWall

#define STATE_FRB (STATE_f && STATE_b && !STATE_l && STATE_r) //backWall
#define STATE_LFR (STATE_r && !STATE_b && STATE_l && STATE_f) //backWall
#define STATE_LFB (!STATE_r && STATE_b && STATE_l && STATE_f) //backWall
#define STATE_LB (!STATE_r && STATE_b && STATE_l && !STATE_f) //backWall
#define STATE_RB (STATE_r && STATE_b && !STATE_l && !STATE_f) //backWall
#define STATE_FR (STATE_r && !STATE_b && !STATE_l && STATE_f) //backWall
#define STATE_LF (!STATE_r && !STATE_b && STATE_l && STATE_f) //backWall
#define STATE_LR (STATE_r && !STATE_b && STATE_l && !STATE_f) //backWall
#define STATE_FB (!STATE_r && STATE_b && !STATE_l && STATE_f) //backWall
#define STATE_F (!STATE_r && !STATE_b && !STATE_l && STATE_f) //backWall
#define STATE_R (STATE_r && !STATE_b && !STATE_l && !STATE_f) //backWall
#define STATE_L (!STATE_r && !STATE_b && STATE_l && !STATE_f) //backWall
#define STATE_B (!STATE_r && STATE_b && !STATE_l && !STATE_f) //backWall


#define STATE_NONE (!STATE_r && !STATE_b && !STATE_l && !STATE_f) //backWall

#include <math.h>

#include <ros/ros.h>

#include <differential_drive/Speed.h>
#include <motor_control/Navigation.h>
#include <motor_control/NavigationStatus.h>
#include <ir_obstacle_detection/IRObstacleSignal.h>
#include <robot_utilities/ENavigationMode.h>
#include <odometry_data/Odometry.h>
#include <global_position_estimation/Options.h>

ros::Publisher g_SpeedReferencePub;
ros::Publisher g_NavigationPub;
ros::Publisher g_GlobalPositionEstimationOptionPub;

ros::Subscriber g_IRObstacleSignalSub;
ros::Subscriber g_GlobalPositionSub;
ros::Subscriber g_NavigationSub;

ir_obstacle_detection::IRObstacleSignal::ConstPtr g_pIRData;
odometry_data::Odometry::ConstPtr g_pGlobalPosition;
motor_control::NavigationStatus::ConstPtr g_pNavigationStatus;

enum RobotMode
{
    IDLE,
    MAPPING,
    NAVIGATION
} g_eRobotMode;

enum MappingNode
{
    EXPLORING,
    STOPPING,
    TURNING,
    ENTERING_FLOOR
} g_eMappingMode;

enum Move
{
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD
} g_eLastMove;

void receiveIRObstacleSignal(const ir_obstacle_detection::IRObstacleSignal::ConstPtr& pMsg)
{
    g_pIRData = pMsg;
}

void receiveGlobalPosition(const odometry_data::Odometry::ConstPtr& pMsg)
{
    g_pGlobalPosition = pMsg;
}

void receiveNavigationStatus(const motor_control::NavigationStatus::ConstPtr& pMsg)
{
    g_pNavigationStatus = pMsg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");
    ros::NodeHandle n;

    g_SpeedReferencePub = n.advertise<differential_drive::Speed>("/motor_control/SpeedReference", 1);
    g_NavigationPub = n.advertise<motor_control::Navigation>("/motor_control/Navigation", 1);
    g_GlobalPositionEstimationOptionPub = n.advertise<global_position_estimation::Options>("/global_position_estimation/Options", 1);

    g_IRObstacleSignalSub = n.subscribe("/ir_obstacle_detection/IRObstacleSignal", 1, receiveIRObstacleSignal);
    g_GlobalPositionSub = n.subscribe("/motion/Odometry", 1, receiveGlobalPosition);
    g_NavigationSub = n.subscribe("/motor_control/NavigationStatus", 1, receiveNavigationStatus);

    ros::Rate loop_rate(100);

    bool bLogged = false;
    float fTurningAngle = 0.0f, fTargetAngle = 0.0f;
    differential_drive::Speed speedReferenceMsg;
    motor_control::Navigation navigationMsg;
    global_position_estimation::Options globalPositionEstimationOptionMsg;

    // give the ir sensors some time to detect the initial state
    ros::Duration(5.0).sleep();

    g_eLastMove = FORWARD;

    while (ros::ok())
    {
        switch (g_eRobotMode)
        {
        case IDLE:
            if (!bLogged)
            {
                ROS_INFO("### IDLE ###");
                bLogged = true;
            }
            /* space for initializations. */
            if (g_pIRData)
            {
                g_eRobotMode = MAPPING;
                bLogged = false;
            }
            break;
        case MAPPING:
            /* Drive around and map. */
            switch (g_eMappingMode)
            {
            case EXPLORING:
                if (!bLogged)
                {
                    ROS_INFO("### EXPLORING ###");
                    bLogged = true;
                }
                /*
                 * Drive forward with low speed.
                 */
                speedReferenceMsg.W1 = 0.18; // [m/s]
                speedReferenceMsg.W2 = 0.18; // [m/s]
                g_SpeedReferencePub.publish(speedReferenceMsg);

                /*
                 * Enable wall alignment.
                 */
                globalPositionEstimationOptionMsg.bActivateThetaImprovement = true;
                g_GlobalPositionEstimationOptionPub.publish(globalPositionEstimationOptionMsg);

                /*
                 * In case of detected wall in front start turning.
                 */
		
                if (STATE_LBR || STATE_LB || STATE_RB || STATE_LR || STATE_L || STATE_B || STATE_NONE || STATE_R)
                {
		    g_eLastMove = FORWARD;
                }
                else
                {
                    ROS_INFO("detected whole in left wall");
                    ros::Duration(1.0).sleep();
                    g_eMappingMode = STOPPING;
                }
		/*
                if (!g_pIRData->front_left && !g_pIRData->rear_left && !g_pIRData->front && !g_pIRData->back)
                {
                    ROS_INFO("detected whole in left wall");
                    g_eMappingMode = STOPPING;
                }
                else if (g_pIRData->front_distance < 0.10)
                {
                    ROS_INFO("detected wall in front");
                    g_eMappingMode = STOPPING;
                }
		*/
                break;
            case STOPPING:
                ROS_INFO("### STOPPING ###");
                bLogged = false;
                /*
                 * Set speed reference to zero.
                 */
                speedReferenceMsg.W1 = 0.0; // [m/s]
                speedReferenceMsg.W2 = 0.0; // [m/s]
                g_SpeedReferencePub.publish(speedReferenceMsg);

                /*
                 * Disable wall alignment.
                 */
                globalPositionEstimationOptionMsg.bActivateThetaImprovement = false;
                g_GlobalPositionEstimationOptionPub.publish(globalPositionEstimationOptionMsg);

                /*
                 * Determine turning angle by detecting free side.
                 */
                if (STATE_FRB || STATE_FR || STATE_FB || STATE_F) // turn left
                {
                    g_eLastMove = LEFT;
                    fTurningAngle = 0.5f * M_PI;
                }
                else if (STATE_LFB || STATE_LF) // turn right
                {
                    g_eLastMove = RIGHT;
                    fTurningAngle = -0.5f * M_PI;
                }
		else if (STATE_LFR)
		{
		    g_eLastMove = BACKWARD;
		    fTurningAngle = -M_PI;
		}
                else // go forward
                {
                    g_eMappingMode = EXPLORING;
                    break;
                }
                /*
                if (g_pIRData->front_left && g_pIRData->front_right)
                    fTurningAngle = M_PI; // turn 180 degrees
                else if (!g_pIRData->front_left && g_pIRData->front_right)
                    fTurningAngle = 0.5f * M_PI; // turn +90 degrees
                else if (g_pIRData->front_left && !g_pIRData->front_right)
                    fTurningAngle = -0.5f * M_PI; // turn -90 degrees
                else if (!g_pIRData->front_left && !g_pIRData->front_right)
                    fTurningAngle = 0.5f * M_PI; // turn +90 degrees
                else
                    break;
                */
                /*
                 * Send turning command.
                 */
                navigationMsg.eNavigationMode = robot_utilities::NAVIGATE_TO_RELATIVE_DIRECTION;
                navigationMsg.fTargetDirection = fTurningAngle;
                navigationMsg.fTargetDistance = 0.0f;
                g_NavigationPub.publish(navigationMsg);

                /*
                 * Continue with turning until target angle is reached.
                 */
                //fTargetAngle = g_pGlobalPosition->theta + fTurningAngle;
                g_eMappingMode = TURNING;
                break;
            case TURNING:
                if (!bLogged)
                {
                    ROS_INFO("### TURNING ###");
                    bLogged = true;
                }
                /*
                 * Get back to exploring mode when turn is completed (== differs not less than 5 degrees from target angle)
                 */
                //ROS_INFO("turning: %g target: %g current: %g", fTurningAngle, fTargetAngle, g_pGlobalPosition->theta);
                //if ((fabs(g_pGlobalPosition->theta) < (fabs(fTargetAngle) + 0.30f))
                //    && (fabs(g_pGlobalPosition->theta) > (fabs(fTargetAngle) - 0.30f)))
                //{
                if (g_pNavigationStatus)
                {
                    if (g_pNavigationStatus->bNavigationCompleted)
                    {
                        ros::Duration(5.0).sleep();
                        /*
                         * Send turning command.
                         */
                        /*navigationMsg.eNavigationMode = robot_utilities::NAVIGATE_TO_RELATIVE_DIRECTION;
                        navigationMsg.fTargetDirection = 0.0;
                        navigationMsg.fTargetDistance = 15.0f;
                        g_NavigationPub.publish(navigationMsg);
			*/
                        g_eMappingMode = EXPLORING;
                        bLogged = false;
                    }
                }
                break;
            case ENTERING_FLOOR:
                if (!bLogged)
                {
                    ROS_INFO("### ENTERING_FLOOR ###");
                    bLogged = true;
                }

                if (g_pNavigationStatus)
                {
                    if (g_pNavigationStatus->bNavigationCompleted)
                    {
                        ros::Duration(2.0).sleep();
                        g_eMappingMode = EXPLORING;
                        bLogged = false;
                    }
                }
                break;
            };
            break;
        case NAVIGATION:
            /* Collect objects that were previously found on the map. */

            break;
        };

        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
