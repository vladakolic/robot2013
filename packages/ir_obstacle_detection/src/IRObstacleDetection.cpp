#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "ros/ros.h"
#undef INT_MAX
#include <differential_drive/AnalogC.h>
//#include <map_creation/ObstacleLine.h>
#include <robot_utilities/RobotProperties.h>
#include <ir_obstacle_detection/IRObstacleSignal.h>

ros::Subscriber g_AnalogCSub;
ros::Publisher g_ObstaclePub;

const float g_fTREAT_AS_OBSTACLE_THRESHOLD = 0.25f;

float g_fDistanceFrontLeft = g_fTREAT_AS_OBSTACLE_THRESHOLD + 1.f;
float g_fDistanceFrontRight = g_fTREAT_AS_OBSTACLE_THRESHOLD + 1.f; 
float g_fDistanceFront = g_fTREAT_AS_OBSTACLE_THRESHOLD + 1.f;
float g_fDistanceBack = g_fTREAT_AS_OBSTACLE_THRESHOLD + 1.f;
float g_fDistanceRearLeft = g_fTREAT_AS_OBSTACLE_THRESHOLD + 1.f;
float g_fDistanceRearRight = g_fTREAT_AS_OBSTACLE_THRESHOLD + 1.f;

float g_fVoltageIRFrontLeftTmp = 0.f;
float g_fVoltageIRFrontRightTmp = 0.f;
float g_fVoltageIRFrontTmp = 0.f;
float g_fVoltageIRBackTmp = 0.f;
float g_fVoltageIRRearLeftTmp = 0.f;
float g_fVoltageIRRearRightTmp = 0.f;

const size_t g_nNumberOfSamples = 5;
size_t g_nCounter = g_nNumberOfSamples; // initialize Tmp values first

struct Line
{
    float m_fX0, m_fY0, m_fZ0, m_fX1, m_fY1, m_fZ1;
};

/**
 * Convert voltage to distance in [m] by the use of a function found in the ir 
 * sensor's manual and optimized in matlab.
 *
 * @param fVoltage
 *  amount of voltage
 */
float convertVoltageToDistance(const float& fVoltage)
{
    float p1, p2, p3, p4, p5;
    p1 = 0.040919f;
    p2 = -0.32525f;
    p3 = 0.94274f;
    p4 = -1.2223f;
    p5 = 0.68749f;
    return p1 * fVoltage*fVoltage*fVoltage*fVoltage
         + p2 * fVoltage*fVoltage*fVoltage
         + p3 * fVoltage*fVoltage
         + p4 * fVoltage
         + p5;
}

/**
 * Receive ir sensor data.
 *
 * @param pMsg
 *  message containing the ir sensor data in channel 3-8
 */
void receiveIRSensorData(const differential_drive::AnalogC::ConstPtr& pMsg)
{
    if (g_nCounter < g_nNumberOfSamples)
    {
        // convert ADC to voltage
        g_fVoltageIRFrontLeftTmp += 0.0048828125f * float(pMsg->ch8);
        g_fVoltageIRFrontRightTmp += 0.0048828125f * float(pMsg->ch5);
        g_fVoltageIRFrontTmp += 0.0048828125f * float(pMsg->ch4);
        g_fVoltageIRBackTmp += 0.0048828125f * float(pMsg->ch3);
        g_fVoltageIRRearLeftTmp += 0.0048828125f * float(pMsg->ch7);
        g_fVoltageIRRearRightTmp += 0.0048828125f * float(pMsg->ch6);

        g_nCounter++;
    }
    else
    {
        float fVoltageIRFrontLeft = g_fVoltageIRFrontLeftTmp / g_nNumberOfSamples;
        float fVoltageIRFrontRight = g_fVoltageIRFrontRightTmp / g_nNumberOfSamples;
        float fVoltageIRFront = g_fVoltageIRFrontTmp / g_nNumberOfSamples;
        float fVoltageIRBack = g_fVoltageIRBackTmp / g_nNumberOfSamples;
        float fVoltageIRRearLeft = g_fVoltageIRRearLeftTmp / g_nNumberOfSamples;
        float fVoltageIRRearRight = g_fVoltageIRRearRightTmp / g_nNumberOfSamples;

        g_fVoltageIRFrontLeftTmp = 0.f;
        g_fVoltageIRFrontRightTmp = 0.f;
        g_fVoltageIRFrontTmp = 0.f;
        g_fVoltageIRBackTmp = 0.f;
        g_fVoltageIRRearLeftTmp = 0.f;
        g_fVoltageIRRearRightTmp = 0.f;

        // convert voltage to distance [m]
        g_fDistanceFrontLeft  = convertVoltageToDistance(fVoltageIRFrontLeft);
        g_fDistanceFrontRight = convertVoltageToDistance(fVoltageIRFrontRight);
        g_fDistanceFront      = convertVoltageToDistance(fVoltageIRFront);
        g_fDistanceBack       = convertVoltageToDistance(fVoltageIRBack);
        g_fDistanceRearLeft   = convertVoltageToDistance(fVoltageIRRearLeft);
        g_fDistanceRearRight  = convertVoltageToDistance(fVoltageIRRearRight);

        // TODO both left and right ir sensor have to be displaced by an equal amount, 
        // i.e. should give roughly the same value for the same distances.
        // This seems to be slightly off right now (right gives values around 1.3 centimeters short atm, why is this?)
        g_nCounter = 0;

            /* 
        ROS_INFO("front left = %g", g_fDistanceFrontLeft);
        ROS_INFO("rear left = %g", g_fDistanceRearLeft);
        ROS_INFO("front right = %g", g_fDistanceFrontRight);
        ROS_INFO("rear right = %g", g_fDistanceRearRight);
        ROS_INFO("front = %g", g_fDistanceFront);
        ROS_INFO("back = %g", g_fDistanceBack);
        */
    }

}


/**
 *
 */
void extractObstacleLine(Line* pLine, const float& fX0, const float& fY0, const float& fZ0, const float& fX1, const float& fY1, const float& fZ1)
{
    pLine = new Line();
    pLine->m_fX0 = fX0;
    pLine->m_fY0 = fY0;
    pLine->m_fZ0 = fZ0;
    pLine->m_fX1 = fX1;
    pLine->m_fY1 = fY1;
    pLine->m_fZ1 = fZ1;
}


/**
 * Publish obastacle lines on topic "/ir_obstacle_detection/ObstacleLine".
 */
/*void publishObstacle(Line* pLine)
{
    if (pLine)
    {
        // Publish obstacle lines
        map_creation::ObstacleLine obstacleMsg;
        obstacleMsg.fX0 = pLine->m_fX0;
        obstacleMsg.fY0 = pLine->m_fY0;
        obstacleMsg.fZ0 = pLine->m_fZ0;
        obstacleMsg.fX1 = pLine->m_fX1;
        obstacleMsg.fY1 = pLine->m_fY1;
        obstacleMsg.fZ1 = pLine->m_fZ1;
        g_ObstaclePub.publish(obstacleMsg);
    
        delete pLine;
    }
}*/


/**
 * Detect obstacles.
 */
void detectObstacles()
{
    
    bool bObstacleFrontLeft  = g_fDistanceFrontLeft  < g_fTREAT_AS_OBSTACLE_THRESHOLD;
    bool bObstacleFrontRight = g_fDistanceFrontRight < g_fTREAT_AS_OBSTACLE_THRESHOLD;
    bool bObstacleFront      = g_fDistanceFront      < g_fTREAT_AS_OBSTACLE_THRESHOLD;
    bool bObstacleBack       = g_fDistanceBack       < g_fTREAT_AS_OBSTACLE_THRESHOLD;
    bool bObstacleRearLeft   = g_fDistanceRearLeft   < g_fTREAT_AS_OBSTACLE_THRESHOLD;
    bool bObstacleRearRight  = g_fDistanceRearRight  < g_fTREAT_AS_OBSTACLE_THRESHOLD;

    float fHypotenuseLeft, fHypotenuseRight; // 0 if no wall was detected
    float fThetaLeft, fThetaRight;

    if (bObstacleFrontLeft + bObstacleRearLeft < 2){
        fHypotenuseLeft = 0;
        fThetaLeft = 0;
    } else {
        float d1md2 = g_fDistanceFrontLeft - g_fDistanceRearLeft;
        fHypotenuseLeft = sqrt(pow(robot_properties::g_fIR_LEFT_FRONT_REAR_DISPLACEMENT_Y,2) + pow(d1md2,2));
        fThetaLeft = -atan(d1md2 / robot_properties::g_fIR_LEFT_FRONT_REAR_DISPLACEMENT_Y);

    }


    if (bObstacleFrontRight + bObstacleRearRight < 2){
        fHypotenuseRight = 0;
        fThetaRight = 0;
    } else {
        float d1md2 = g_fDistanceFrontRight - g_fDistanceRearRight;
        fHypotenuseRight = sqrt(pow(robot_properties::g_fIR_RIGHT_FRONT_REAR_DISPLACEMENT_Y,2) + pow(d1md2,2));
        fThetaRight = atan(d1md2 / robot_properties::g_fIR_RIGHT_FRONT_REAR_DISPLACEMENT_Y);
    }
    /*
    ROS_INFO("left hypotenuse length  = %f", fHypotenuseLeft);
    ROS_INFO("right hypotenuse length = %f", fHypotenuseRight);
    ROS_INFO("theta left  = %f", 360*fThetaLeft/(2*M_PI));
    ROS_INFO("theta right = %f", 360*fThetaRight/(2*M_PI));
    */

    /*
    ROS_INFO("front left : %d", bObstacleFrontLeft);
    ROS_INFO("front right : %d", bObstacleFrontRight);
    ROS_INFO("front: %d", bObstacleFront);
    ROS_INFO("back: %d", bObstacleBack);
    ROS_INFO("rear left : %d", bObstacleRearLeft);
    ROS_INFO("rear right : %d", bObstacleRearRight);
    */
    
    // publish those badass bools
    ir_obstacle_detection::IRObstacleSignal obst_signal;

    obst_signal.front = bObstacleFront;
    obst_signal.front_left = bObstacleFrontLeft;
    obst_signal.rear_left = bObstacleRearLeft;
    obst_signal.front_right = bObstacleFrontRight;
    obst_signal.rear_right = bObstacleRearRight;
    obst_signal.back = bObstacleBack;

    obst_signal.left_theta = fThetaLeft;
    obst_signal.right_theta = fThetaRight;
    obst_signal.left_hypo_length = fHypotenuseLeft;
    obst_signal.right_hypo_length = fHypotenuseRight;

    obst_signal.front_distance = g_fDistanceFront;
    obst_signal.front_left_distance = g_fDistanceFrontLeft;
    obst_signal.rear_left_distance = g_fDistanceRearLeft;
    obst_signal.front_right_distance = g_fDistanceFrontRight;
    obst_signal.rear_right_distance = g_fDistanceRearRight;
    obst_signal.back_distance = g_fDistanceBack;

    g_ObstaclePub.publish(obst_signal);
    //Line* pLine = (Line*) 0;

    /*
     * Detect obstacles on the robot's left side.
     */

    /*
    if (bObstacleFrontLeft && bObstacleCenterLeft && bObstacleRearLeft)         // FxxxxxxxxxR
    {
        extractObstacleLine(pLine, 
                            (g_fDistanceFrontLeft + g_fDistanceFront + g_fDistanceRearLeft) * .33333333f,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Z,
                            (g_fDistanceFrontLeft + g_fDistanceFront + g_fDistanceRearLeft) * .33333333f,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
    }
    
    else if (bObstacleFrontLeft && !bObstacleRearLeft)   // Fxxxxxx---R
    {
        extractObstacleLine(pLine,
                            (g_fDistanceFrontLeft + g_fDistanceFront) * .5f,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Z,
                            (g_fDistanceFrontLeft + g_fDistanceFront) * .5f,
                            robot_properties::g_fIR_FRONT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
    }
    else if (bObstacleFrontLeft && !bObstacleFront && !bObstacleRearLeft)  // Fxxx------R
    {
        extractObstacleLine(pLine,
                            g_fDistanceFrontLeft,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Z,
                            g_fDistanceFrontLeft,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
                            
    }
    
    else if (bObstacleFrontLeft && !bObstacleCenterLeft && bObstacleRearLeft)   // Fxxx---xxxR
    {
        extractObstacleLine(pLine,
                            (g_fDistanceFrontLeft + g_fDistanceRearLeft) * .5f,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Z,
                            (g_fDistanceFrontLeft + g_fDistanceRearLeft) * .5f,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);

        extractObstacleLine(pLine,
                            (g_fDistanceFrontLeft + g_fDistanceRearLeft) * .5f,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Z,
                            (g_fDistanceFrontLeft + g_fDistanceRearLeft) * .5f,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
                            
                            
    }
    else if (!bObstacleFrontLeft && !bObstacleFront && bObstacleRearLeft)  // F------xxxR
    {
        extractObstacleLine(pLine,
                            g_fDistanceRearLeft,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Z,
                            g_fDistanceRearLeft,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
    }
    else if (!bObstacleFrontLeft && bObstacleFront && bObstacleRearLeft)   // F---xxxxxxR
    {
        extractObstacleLine(pLine,
                            (g_fDistanceFront + g_fDistanceRearLeft) * .5f,
                            robot_properties::g_fIR_FRONT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_SENSOR_DISPLACEMENT_Z,
                            (g_fDistanceFront + g_fDistanceRearLeft) * .5f,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
    }
    */


    /*
     * Detect obstacles on the robot's right side.
     */
     /*
    if (bObstacleFrontRight && bObstacleBack && bObstacleRearRight)         // FxxxxxxxxxR
    {
        extractObstacleLine(pLine, 
                            (g_fDistanceFrontRight + g_fDistanceBack + g_fDistanceRearRight) * .33333333f,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Z,
                            (g_fDistanceFrontRight + g_fDistanceBack + g_fDistanceRearRight) * .33333333f,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
    }
    else if (bObstacleFrontRight && bObstacleBack && !bObstacleRearRight)   // Fxxxxxx---R
    {
        extractObstacleLine(pLine,
                            (g_fDistanceFrontRight + g_fDistanceBack) * .5f,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Z,
                            (g_fDistanceFrontRight + g_fDistanceBack) * .5f,
                            robot_properties::g_fIR_BACK_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_BACK_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
    }
    else if (bObstacleFrontRight && !bObstacleBack && !bObstacleRearRight)  // Fxxx------R
    {
        extractObstacleLine(pLine,
                            g_fDistanceFrontRight,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Z,
                            g_fDistanceFrontRight,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
                            
    }
    else if (bObstacleFrontRight && !bObstacleBack && bObstacleRearRight)   // Fxxx---xxxR
    {
        extractObstacleLine(pLine,
                            (g_fDistanceFrontRight + g_fDistanceRearRight) * .5f,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Z,
                            (g_fDistanceFrontRight + g_fDistanceRearRight) * .5f,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);

        extractObstacleLine(pLine,
                            (g_fDistanceFrontRight + g_fDistanceRearRight) * .5f,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Z,
                            (g_fDistanceFrontRight + g_fDistanceRearRight) * .5f,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
                            
                            
    }
    else if (!bObstacleFrontRight && !bObstacleBack && bObstacleRearRight)  // F------xxxR
    {
        extractObstacleLine(pLine,
                            g_fDistanceRearRight,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Z,
                            g_fDistanceRearRight,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
    }
    else if (!bObstacleFrontRight && bObstacleBack && bObstacleRearRight)   // F---xxxxxxR
    {
        extractObstacleLine(pLine,
                            (g_fDistanceBack + g_fDistanceRearRight) * .5f,
                            robot_properties::g_fIR_BACK_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_BACK_SENSOR_DISPLACEMENT_Z,
                            (g_fDistanceBack + g_fDistanceRearRight) * .5f,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Y,
                            robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Z);
        publishObstacle(pLine);
    }
    */
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ir_obstacle_detection");

    ros::NodeHandle n;

    // Connect to mapping node
    //g_ObstaclePub = n.advertise<map_creation::ObstacleLine>("/ir_obstacle_detection/ObstacleLine", 10);
    g_ObstaclePub = n.advertise<ir_obstacle_detection::IRObstacleSignal>("/ir_obstacle_detection/IRObstacleSignal",1);

    // Connect to arduino for ir sensor data
    g_AnalogCSub = n.subscribe("/sensors/ADC", 1, receiveIRSensorData);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        // detect obstacles
        detectObstacles();
        
        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
