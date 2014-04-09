#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "ros/ros.h"
#undef INT_MAX
#include <motor_control/FloatEncoders.h>
#include <motor_control/Navigation.h>
#include <motor_control/NavigationStatus.h>
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Speed.h>
#include <differential_drive/Params.h>
//#include <differential_drive/Odometry.h>
#include <odometry_data/Odometry.h>
#include <keyboard_control/KeyboardStates.h>
#include <robot_utilities/PIDController.h>
#include <robot_utilities/PIDGains.h>
#include <robot_utilities/PIDGainControl.h>
#include <robot_utilities/RobotProperties.h>
#include <robot_utilities/RisingEdgeDetector.h>
#include <robot_utilities/ENavigationMode.h>

using namespace motor_control;

ros::Publisher g_PWMPub;
ros::Publisher g_SpeedPub;
ros::Publisher g_SpeedReferencePub;
ros::Publisher g_ParamsPub;
ros::Publisher g_NavigationPub;
ros::Subscriber g_KeyboardStatesSub;
ros::Subscriber g_PIDGainsSub;
ros::Subscriber g_EncoderSub;
ros::Subscriber g_OdometrySub;
ros::Subscriber g_SpeedReferenceSub;
ros::Subscriber g_NavigationSub;

/** Timestamp from last encoder message. */
ros::Time g_EncodersTimestamp;

/** Timestamp from last speed message. */
ros::Time g_SpeedTimestamp;

/** Timestamp for the navigation controller. */
ros::Time g_NavigationTimestamp;

/** Timestamp for the orientation controller. */
ros::Time g_OrientationTimestamp;

/** Timestamp for the distance controller. */
ros::Time g_DistanceTimestamp;


/** the definition of the loop rate of the motor_control node. */
const float g_fLOOP_RATE= 42.f;
/** the definition of the reference speed for both wheels. */
const float g_fMETERS_PER_SECOND = 0.15f;

robot_utilities::PIDController g_SpeedControllerLeft;
robot_utilities::PIDController g_SpeedControllerRight;
robot_utilities::PIDController g_OrientationController;
robot_utilities::PIDController g_DistanceController;

/** the reference speed of the left wheel. */
float g_fSpeedReferenceLeft = 0.f;
/** the actual speed of the left wheel. */
float g_fSpeedLeft = 0.f;
/** the reference speed of the right wheel. */
float g_fSpeedReferenceRight = 0.f;
/** the actual speed of the right wheel. */
float g_fSpeedRight = 0.f;

/** the global x position of the robot. */
float g_fPositionX = 0.f;
/** the global y position of the robot. */
float g_fPositionY = 0.f;
/** the global orientation of the robot. */
float g_fOrientation = 0.f;

/** the global start x position of the robot in navigation mode. */
float g_fStartPositionX = 0.f;
/** the global start y position of the robot in navigation mode. */
float g_fStartPositionY = 0.f;

/** the target orientation used in navigation mode. */
float g_fTargetOrientation = 0.f;
/** the target distance used in navigation mode. */
float g_fTargetDistance = 0.f;

/** the actually travelled distance in navigation mode. */
float g_fDistance = 0.f;

/** a trigger to start the navigation mode state machine. */
bool g_bNavigationModeActivated = false;
/** a rising edge detector for triggering the navigation mode. */
robot_utilities::RisingEdgeDetector g_RisingEdgeNavigationMode;

/** flag for enabling keyboard control. */
bool g_bKeyControlActivated = true;
/** flag for enabling speed regulation. */
bool g_bSpeedRegulationActivated = true;
/** flag for enabling odometry regulation. */
bool g_bOrientationRegulationActivated = true;
/** flag for enabling distance regulation. */
bool g_bDistanceRegulationActivated = false;


/* rising edge detectors for option keys */
robot_utilities::RisingEdgeDetector g_RisingEdgeK;
robot_utilities::RisingEdgeDetector g_RisingEdgeD;
robot_utilities::RisingEdgeDetector g_RisingEdgeO;
robot_utilities::RisingEdgeDetector g_RisingEdgeS;
robot_utilities::RisingEdgeDetector g_RisingEdgeN;
robot_utilities::RisingEdgeDetector g_RisingEdgeF;
robot_utilities::RisingEdgeDetector g_RisingEdgeQ;
robot_utilities::RisingEdgeDetector g_RisingEdgeW;


void evaluateConfigCommands(const bool& bK, const bool& bD, const bool& bO, const bool& bS)
{
    if (g_bKeyControlActivated && bK)
    {
        g_bKeyControlActivated = false;
        ROS_INFO("Key control deactivated!");
    }
    else if (!g_bKeyControlActivated && bK)
    {
        g_bKeyControlActivated = true;
        ROS_INFO("Key control activated!");
    }

    if (g_bDistanceRegulationActivated && bD)
    {
        g_bDistanceRegulationActivated = false;
        ROS_INFO("Distance controlling deactivated!");
    }
    else if (!g_bDistanceRegulationActivated && bD)
    {
        g_bDistanceRegulationActivated = true;
        ROS_INFO("Distance controlling activated!");
    }

    if (g_bOrientationRegulationActivated && bO)
    {
        g_bOrientationRegulationActivated = false;
        ROS_INFO("Orientation controlling deactivated!");
    }
    else if (!g_bOrientationRegulationActivated && bO)
    {
        g_bOrientationRegulationActivated = true;
        ROS_INFO("Orientation controlling activated!");
    }

    if (g_bSpeedRegulationActivated && bS)
    {
        g_bSpeedRegulationActivated = false;
        ROS_INFO("Speed regulation deactivated!");
    }
    else if (!g_bSpeedRegulationActivated && bS)
    {
        g_bSpeedRegulationActivated = true;
        ROS_INFO("Speed regulation activated!");
    }
}


/**
 * This is a service that allows us to control the speed controller gains with an external python gui.
 */
bool controlSpeedLeftGain(robot_utilities::PIDGainControl::Request& req,
                          robot_utilities::PIDGainControl::Response& res)
{
    g_SpeedControllerLeft.setGains(req.fKP, req.fKI, req.fKD);
    g_SpeedControllerLeft.getGains(res.fKP, res.fKI, res.fKD);
    return true;
}

/**
 * This is a service that allows us to control the speed controller gains with an external python gui.
 */
bool controlSpeedRightGain(robot_utilities::PIDGainControl::Request& req,
                           robot_utilities::PIDGainControl::Response& res)
{
    g_SpeedControllerRight.setGains(req.fKP, req.fKI, req.fKD);
    g_SpeedControllerRight.getGains(res.fKP, res.fKI, res.fKD);
    return true;
}

/**
 * This is a service that allows us to control the speed controller gains with an external python gui.
 */
bool controlOrientationGain(robot_utilities::PIDGainControl::Request& req,
                            robot_utilities::PIDGainControl::Response& res)
{
    g_OrientationController.setGains(req.fKP, req.fKI, req.fKD);
    g_OrientationController.getGains(res.fKP, res.fKI, res.fKD);
    return true;
}

/**
 * This is a service that allows us to control the speed controller gains with an external python gui.
 */
bool controlDistanceGain(robot_utilities::PIDGainControl::Request& req,
                         robot_utilities::PIDGainControl::Response& res)
{
    g_DistanceController.setGains(req.fKP, req.fKI, req.fKD);
    g_DistanceController.getGains(res.fKP, res.fKI, res.fKD);
    return true;
}


/**
 * This function is used whenever there is the need of a fDeltaT. It outputs a warning if a certain limit is exceeded.
 */
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
 * This function sends pwm signals to the motors. It is continuously executed in the main loop of this package. The 
 * actual pwm signal is computed on the basis of the current speed reference, a global variable that can be set 
 * from arbitrarily parts of this file, e.g. in the keyboard control.
 *
 * ! Notice that the frequency at which the pwm signal is published - that will be the frequency of the main loop of this
 *   node - defines the frequency at which encoder updates are received, since an encoder message is triggered by the 
 *   receiving of a speed message at the arduino.
 */
void sendSpeedReference()
{
    /*
     * Calculate the timegap (in [sec]) between this and the last call of this function.
     */
    float fDeltaT = calculateDeltaT(g_SpeedTimestamp, 1.0f / (g_fLOOP_RATE+1.0));

	differential_drive::PWM pwmMsg;

    if (g_bSpeedRegulationActivated)
    {
        /*
         * If the speed regulation is active we continuosly feed the related pid-controller with the current speed
         * reference and the actually measured speed. The controller's output is then applied to the motors in form
         * of a pwm signal.
         */
        g_SpeedControllerLeft.reset(g_fSpeedReferenceLeft, 0.0, -100.f, 100.f);
        pwmMsg.PWM1 = round(g_SpeedControllerLeft.update(g_fSpeedLeft, fDeltaT));
        pwmMsg.PWM1 = (pwmMsg.PWM1 > 255 ? 255 : (pwmMsg.PWM1 < -255 ? -255 : pwmMsg.PWM1));
        g_SpeedControllerRight.reset(g_fSpeedReferenceRight, 0.0, -100.f, 100.f);
        pwmMsg.PWM2 = -round(g_SpeedControllerRight.update(g_fSpeedRight, fDeltaT)); // negate because of motor mounting
        pwmMsg.PWM2 = (pwmMsg.PWM2 > 255 ? 255 : (pwmMsg.PWM2 < -255 ? -255 : pwmMsg.PWM2));
    }
    else
    {
        /*
         * If the speed regulation is turned off no speed must be send to the motors.
         */
        pwmMsg.PWM1 = 0;
        pwmMsg.PWM2 = 0;
    }

    /*
     * Publish the pwm signal (that will be received by the motors).
     */
    pwmMsg.header.stamp = g_SpeedTimestamp;
    g_PWMPub.publish(pwmMsg);

    /*
     * Publish the speed reference. This will be quite useful to be plotted when tuning the speed controller.
     */
    differential_drive::Speed speedReferenceMsg;
    speedReferenceMsg.W1 = g_fSpeedReferenceLeft;
    speedReferenceMsg.W2 = g_fSpeedReferenceRight;
    speedReferenceMsg.header.stamp = g_SpeedTimestamp;
    g_SpeedReferencePub.publish(speedReferenceMsg);

    /*
     * Finally reset the speed reference and allow the other functions to contribute a speed reference.
     */
    g_fSpeedReferenceLeft = 0.f;
    g_fSpeedReferenceRight = 0.f;
}

/**
 * Calculate a speed reference based on the current state of the keyboard's arrow keys. 
 *
 * @param bLeft
 *  state of the left arrow key
 * @param bRight
 *  state of the right arrow key
 * @param bUp
 *  state of the up arrow key
 * @param bDown
 *  state of the down arrow key
 */
void evaluateSpeedCommands(const bool& bLeft, const bool& bRight, const bool& bUp, const bool& bDown)
{
    if(g_bKeyControlActivated)
    {
        g_fSpeedReferenceLeft += g_fMETERS_PER_SECOND * (bDown || bLeft ? -1 : (bUp || bRight ? 1 : 0)); 
        g_fSpeedReferenceRight += g_fMETERS_PER_SECOND * (bDown || bRight ? -1 : (bUp || bLeft ? 1 : 0));
    }
}

/**
 * Set the orientation reference based on the current state of the related keys. 
 *
 */
void evaluateOrientationCommands(const bool& bOption)
{
    if(g_bKeyControlActivated)
    {
       // g_fTargetOrientation = bOption ? 0.5f * M_PI : 0.0f;
        ROS_INFO("orientation ref: %g", g_fTargetOrientation);
    }
}

/**
 * Test the navigation routine navigateRobot.
 */
void evaluateNavigationCommands(const bool& bOption)
{
    if (bOption)
    {
        g_fTargetOrientation = 0.25f * M_PI;
        g_fTargetDistance = 0.2f;
        g_bNavigationModeActivated = true;
    }
}


void evaluateTurnCommands(const bool& bLeft, const bool& bRight){
     

    if (bLeft){ g_fTargetOrientation -= 0.5*M_PI;}

    if (bRight){ g_fTargetOrientation += 0.5*M_PI;}


}


/**
 * Receive speed reference from external node.
 */
void receiveSpeedReference(const differential_drive::Speed::ConstPtr& pMsg)
{
    g_fSpeedReferenceLeft += pMsg->W1;
    g_fSpeedReferenceRight += pMsg->W2;
}

/**
 * Receive encoder values from the motors. Notice that there is a filter node in between the motors and this node. That
 * leads to a message type of motor_control::FloatEncoders (float) instead of diffential_drive::Encoders (int).
 * 
 * ! Notice that the frequency of receiving encoder updates is related to the frequency pwm signals are send to the 
 *   motors.
 *
 * @param pMsg
 *  a message that contains the mean encoder change in [ticks]
 */
void receiveEncoders(const motor_control::FloatEncoders::ConstPtr& pMsg)
{
    /*
     * Calculate the timegap (in [sec]) between this and the last call of this function.
     */
    float fDeltaT = calculateDeltaT(g_EncodersTimestamp, 1.0f / (g_fLOOP_RATE+1.0));
   
    // calculate speed in [m/sec]
    g_fSpeedLeft = (2. * M_PI * robot_properties::g_fRadiusLeft * -pMsg->fDeltaEncoder1)
                 / (robot_properties::g_fTicksPerRev * fDeltaT);
    g_fSpeedRight = (2. * M_PI * robot_properties::g_fRadiusRight * -pMsg->fDeltaEncoder2)
                  / (robot_properties::g_fTicksPerRev * fDeltaT);
    
    differential_drive::Speed speedMsg;
    speedMsg.W1 = g_fSpeedLeft;
    speedMsg.W2 = g_fSpeedRight;
    speedMsg.header.stamp = g_EncodersTimestamp;
    g_SpeedPub.publish(speedMsg);
}

/**
 * Define directions and relate them to discrete orientation angles.
 */
enum EDirection
{
    NORTH = 0,
    WEST = 90,
    SOUTH = 180,
    EAST = 270
};

/**
 * Define states of a state machine that navigates to a given distance in a given direction.
 */
enum ENavigationState
{
    BEGIN_NAVIGATION,
    INIT_ORIENTATION_CONTROLLER,
    CONTROL_ORIENTATION,
    INIT_DISTANCE_CONTROLLER,
    CONTROL_DISTANCE,
    END_NAVIGATION
};

/** The current state of the navigation program. */
ENavigationState g_eNavigationState = BEGIN_NAVIGATION;

/**
 * Receive a navigation message that tells the robot where to move.
 *
 * @param pMsg
 *  a message containing a navigation mode and its parameters
 */
void receiveNavigation(const motor_control::Navigation::ConstPtr& pMsg)
{
    float fMovementVectorX, fMovementVectorY;

    /*
     * Based on the navigation mode that is contained in the message the navigation state machine is started in a 
     * different way.
     */
    robot_utilities::ENavigationMode eNavigationMode = robot_utilities::ENavigationMode(pMsg->eNavigationMode);
    switch (eNavigationMode)
    {
    case robot_utilities::NAVIGATE_TO_TARGET_POSITION:
        /*
         * Calculate an orientation and a distance by comparing the actual with the targeted position.
         */
        fMovementVectorX = pMsg->fTargetPositionX - g_fPositionX;
        fMovementVectorY = pMsg->fTargetPositionY - g_fPositionY;

        // obtain a relative orientation change by calculating the orientation of the movement vector
        g_fTargetOrientation = tan(-fMovementVectorX / fMovementVectorY) + g_fOrientation;
        // obtain the target distance by calculating the length of the movement vector
        g_fTargetDistance = sqrt(fMovementVectorX*fMovementVectorX + fMovementVectorY*fMovementVectorY);
        break;
    case robot_utilities::NAVIGATE_TO_ABSOLUTE_DIRECTION:
        /*
         * Just copy the parameters.
         */
        g_fTargetOrientation = pMsg->fTargetDirection;
        g_fTargetDistance = pMsg->fTargetDistance;
        break;
    case robot_utilities::NAVIGATE_TO_RELATIVE_DIRECTION:
        /*
         * Convert the relative angle to a global angle.
         */
        g_fTargetOrientation = pMsg->fTargetDirection + g_fOrientation;
        g_fTargetDistance = pMsg->fTargetDistance;
        break;
    }

    g_bNavigationModeActivated = true;
}

/**
 * Navigate the robot based on orientation and distance.
 */
void navigateRobot()
{
    /*
     * At first regulate so that the target orientation is achieved and don't regulate the distance.
     */
    if (g_fOrientation > (g_fTargetOrientation + 0.05) || g_fOrientation < (g_fTargetOrientation - 0.05))
    {
        /* Set the navigation status to be not finished */
        motor_control::NavigationStatus statusMsg;
        statusMsg.bNavigationCompleted = false;
        g_NavigationPub.publish(statusMsg);

        g_bOrientationRegulationActivated = true;
        g_bDistanceRegulationActivated = false;
    }
    /*
     * Keep on regulating the target orientation while also regulating the target distance.
     */
    else if (g_fDistance > (g_fTargetDistance + 0.03) || g_fDistance < (g_fTargetDistance - 0.03))
    {
        g_bOrientationRegulationActivated = true;
        g_bDistanceRegulationActivated = true;
    }
    /*
     * If both the target orientation and distance are reached according to a certain level of precision switch off 
     * the distance regulation and also the navigation mode but keep the orientation regulation running.
     */
    else
    {
        /* Set the navigation status to be not finished */
        motor_control::NavigationStatus statusMsg;
        statusMsg.bNavigationCompleted = true;
        g_NavigationPub.publish(statusMsg);
        
        g_bOrientationRegulationActivated = true;
        g_bDistanceRegulationActivated = false;
        g_bNavigationModeActivated = false;
    }
}

/**
 * Control the global orientation angle of the robot.
 *
 * @param fValue
 *  the current measurement of the orientation angle
 * @param fReference
 *  the reference orientation angle to regulate to
 */
void controlOrientation(const float& fValue, const float& fReference)
{
    g_OrientationController.reset(fReference, 0.001f);

    float fDeltaT = calculateDeltaT(g_OrientationTimestamp, 1.0f / (g_fLOOP_RATE + 1.0));
    float fUpdate = g_OrientationController.update(fValue, fDeltaT);

    // cut the update at the overall speed reference
    if (fUpdate > g_fMETERS_PER_SECOND)
        fUpdate = g_fMETERS_PER_SECOND;
    else if (fUpdate < -g_fMETERS_PER_SECOND)
        fUpdate = -g_fMETERS_PER_SECOND;

    g_fSpeedReferenceLeft += -fUpdate;
    g_fSpeedReferenceRight += fUpdate;
}

/**
 * Measure the currently travelled distance.
 */
float measureCurrentDistance(const float& fCurrentX, const float& fCurrentY, const float& fInitialX, const float& fInitialY)
{
    // Span a vector pointing from the start position to the current position 
    float fDeltaX = fCurrentX - fInitialX;
    float fDeltaY = fCurrentY - fInitialY;

    // Measure the currently travelled distance as length of the vector above
    return sqrt(fDeltaX*fDeltaX + fDeltaY*fDeltaY);
}

/**
 * Control the distance travelled.
 */
void controlDistance(const float& fCurrentX, const float& fCurrentY, const float& fReference)
{
    g_DistanceController.reset(fReference, 0.0001f);

    g_fDistance = measureCurrentDistance(fCurrentX, fCurrentY, g_fStartPositionX, g_fStartPositionY);

    float fDeltaT = calculateDeltaT(g_DistanceTimestamp, 1.0f / (g_fLOOP_RATE + 1.0));
    float fUpdate = g_DistanceController.update(g_fDistance, fDeltaT);

    // cut the update at the overall speed reference
    if (fUpdate > g_fMETERS_PER_SECOND)
        fUpdate = g_fMETERS_PER_SECOND;
    else if (fUpdate < -g_fMETERS_PER_SECOND)
        fUpdate = -g_fMETERS_PER_SECOND;

    g_fSpeedReferenceLeft += fUpdate;
    g_fSpeedReferenceRight += fUpdate;
}

/**
 * Receive odometry values from the global position estimation node.
 *
 * @param pMsg
 *  a message containing the odometry data - position x, y and orientation theta
 */
void receiveOdometry(const odometry_data::Odometry::ConstPtr& pMsg)
{
    g_fPositionX = pMsg->x;
    g_fPositionY = pMsg->y;
    g_fOrientation = pMsg->theta;

    if (g_bOrientationRegulationActivated)
    {
        controlOrientation(pMsg->theta, g_fTargetOrientation);
    }
    if (g_bDistanceRegulationActivated)
    {
        controlDistance(pMsg->x, pMsg->y, g_fTargetDistance);
    }
}

/**
 * Receive and evaluate keyboard states.
 */
void receiveKeyboardStates(const keyboard_control::KeyboardStates::ConstPtr& pMsg)
{
    evaluateConfigCommands(g_RisingEdgeK(pMsg->bK), g_RisingEdgeD(pMsg->bD), g_RisingEdgeO(pMsg->bO), g_RisingEdgeS(pMsg->bS));
    evaluateSpeedCommands(pMsg->bLeftArrow, pMsg->bRightArrow, pMsg->bUpArrow, pMsg->bDownArrow);
    //evaluateOrientationCommands(pMsg->bR);
    evaluateNavigationCommands(g_RisingEdgeN(pMsg->bN));
    evaluateTurnCommands(g_RisingEdgeQ(pMsg->bQ), g_RisingEdgeW(pMsg->bW));
}


/**
 * This is the main function of this node that contains the main loop.
 *
 * ! Notice that its runtime frequency influences the sending of pwm signals to the motors and thus also the receiving
 *   of encoder updates.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "MotorControl");

    ros::NodeHandle n;

    g_EncodersTimestamp = ros::Time::now();
    g_SpeedTimestamp = ros::Time::now();
    g_OrientationTimestamp = ros::Time::now();
    g_DistanceTimestamp = ros::Time::now();

    // Initialize publishers
    g_PWMPub = n.advertise<differential_drive::PWM>("/motion/PWM", 1);
    g_SpeedPub = n.advertise<differential_drive::Speed>("/motion/Speed", 1);
    g_SpeedReferencePub = n.advertise<differential_drive::Speed>("/motion/SpeedReference", 1);
    g_ParamsPub = n.advertise<differential_drive::Params>("/motors/params", 1);
    g_NavigationPub = n.advertise<motor_control::NavigationStatus>("/motor_control/NavigationStatus", 1);

    // Initialize subscribers
    g_KeyboardStatesSub = n.subscribe("/keyboard_control/KeyboardStates", 1, receiveKeyboardStates);
    g_EncoderSub = n.subscribe("/motion/FilteredEncoders", 1, receiveEncoders);
	//g_OdometrySub = n.subscribe("/motion/Odometry", 1, receiveOdometry);
	g_OdometrySub = n.subscribe("/motion/Odometry", 1, receiveOdometry);
    g_SpeedReferenceSub = n.subscribe("/motor_control/SpeedReference", 1, receiveSpeedReference);
    g_NavigationSub = n.subscribe("/motor_control/Navigation", 1, receiveNavigation);

    // Initialize services
    ros::ServiceServer speedLeftGainSrv = n.advertiseService("/motor_control/SpeedLeftGainControl", controlSpeedLeftGain);
    ros::ServiceServer speedRightGainSrv = n.advertiseService("/motor_control/SpeedRightGainControl", controlSpeedRightGain);
    ros::ServiceServer orientationGainSrv = n.advertiseService("/motor_control/OrientationGainControl", controlOrientationGain);
    ros::ServiceServer distanceGainSrv = n.advertiseService("/motor_control/DistanceGainControl", controlDistanceGain);

    // should lead to 4 tics on the encoder measurements at final speed of 0.15 m/s
    ros::Rate loop_rate(g_fLOOP_RATE);

    // initialize controller gains
    g_SpeedControllerLeft.setGains(350., 800., 10.);
    g_SpeedControllerRight.setGains(320., 800., 10.);
    g_DistanceController.setGains(5., 0., 0.);
    g_OrientationController.setGains(0.6, 0.0, 0.05);//(0.2, 0.02, 0.05);

    while (ros::ok())
    {
        // run the speed regulation to control the motors
        sendSpeedReference();

        if (g_bNavigationModeActivated)
            navigateRobot();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
