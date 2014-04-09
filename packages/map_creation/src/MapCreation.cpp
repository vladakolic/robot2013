#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <tr1/memory>

#include "ros/ros.h"
#include <ir_obstacle_detection/IRObstacleSignal.h>
#include <odometry_data/Odometry.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <robot_utilities/RobotProperties.h>
#include <map_creation/MapModel.h>
#include <map_creation/Map.h>
//#include <map_creation/MapData.h>

const int g_fMAP_WIDTH = 5.0f;
const int g_fMAP_HEIGHT = 5.0f;
const float g_fCELL_SIZE = 0.05;
const float g_fTREAT_AS_OBSTACLE_THRESHOLD = 0.25f + robot_properties::g_fEDGE_TO_CENTER_DISTANCE_X;

ros::Subscriber g_GlobalPositionEstimationSub;
ros::Subscriber g_IRObstacleDetectionSub;
ros::Publisher g_MapPub;

float g_fGlobalRobotX;
float g_fGlobalRobotY;
float g_fGlobalRobotTheta;

ir_obstacle_detection::IRObstacleSignal::ConstPtr g_pIRData;

MapModel g_map;

const int g_iACCUMULATOR_THRESHOLD = 10;
std::vector< std::pair<float, float> > g_vLeftWallAccumulator;
std::vector< std::pair<float, float> > g_vRightWallAccumulator;

void multiplyPointWithMatrix(float* pPoint, float* const pMatrix)
{
    /*
     * a_11  a_21  a_31     x     x'
     * a_12  a_22  a_32  x  y  =  y'
     * a_13  a_23  a_33     w     w'
     */
    float fX = pPoint[0], fY = pPoint[1], fW = pPoint[2];
    pPoint[0] = pMatrix[0] * fX + pMatrix[3] * fY + pMatrix[6] * fW;
    pPoint[1] = pMatrix[1] * fX + pMatrix[4] * fY + pMatrix[7] * fW;
    pPoint[2] = pMatrix[2] * fX + pMatrix[5] * fY + pMatrix[8] * fW;
}

void create2DHomogeneousTransformationMatrix(float* pMatrix, float fRotation, float fTranslationX, float fTranslationY)
{
    /*
     * [0] [3] [6]   cos theta  -sin theta  t_x   a_11  a_21  a_31
     * [1] [4] [7] = sin theta   cos theta  t_y = a_12  a_22  a_32
     * [2] [5] [8]       0           0       1    a_13  a_23  a_33
     */
    pMatrix[0] = cos(fRotation);
    pMatrix[3] = -sin(fRotation);
    pMatrix[6] = fTranslationX;
    pMatrix[1] = sin(fRotation);
    pMatrix[4] = cos(fRotation);
    pMatrix[7] = fTranslationY;
    pMatrix[2] = 0.0f;
    pMatrix[5] = 0.0f;
    pMatrix[8] = 1.0f;
}

void detectObstacles(){
    if (g_pIRData)
    {
        // position in 2D homogeneous coords
        float ptObstacle[3];

        // transformation from robot to world space
        float matWorldTRobot[9];
        create2DHomogeneousTransformationMatrix(&matWorldTRobot[0], g_fGlobalRobotTheta, g_fGlobalRobotX, g_fGlobalRobotY);

        if (g_pIRData->front_left)
        {
            ptObstacle[0] = robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_X - g_pIRData->front_left_distance;
            ptObstacle[1] = robot_properties::g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Y;
            ptObstacle[2] = 1.0;

            multiplyPointWithMatrix(&ptObstacle[0], &matWorldTRobot[0]);

            g_map.addObstacle(ptObstacle[0], ptObstacle[1]);
        }
        if (g_pIRData->rear_left)
        {
            ptObstacle[0] = robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_X - g_pIRData->rear_left_distance;
            ptObstacle[1] = robot_properties::g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Y;
            ptObstacle[2] = 1.0;

            multiplyPointWithMatrix(&ptObstacle[0], &matWorldTRobot[0]);

            g_map.addObstacle(ptObstacle[0], ptObstacle[1]);
        }
        if (g_pIRData->front_right)
        {
            ptObstacle[0] = robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_X + g_pIRData->front_right_distance;
            ptObstacle[1] = robot_properties::g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Y;
            ptObstacle[2] = 1.0;

            multiplyPointWithMatrix(&ptObstacle[0], &matWorldTRobot[0]);

            g_map.addObstacle(ptObstacle[0], ptObstacle[1]);
        }
        if (g_pIRData->rear_right)
        {
            ptObstacle[0] = robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_X + g_pIRData->rear_right_distance;
            ptObstacle[1] = robot_properties::g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Y;
            ptObstacle[2] = 1.0;

            multiplyPointWithMatrix(&ptObstacle[0], &matWorldTRobot[0]);

            g_map.addObstacle(ptObstacle[0], ptObstacle[1]);
        }
    }
}

void receiveGlobalPositionEstimation(const odometry_data::Odometry::ConstPtr& pMsg){
	g_fGlobalRobotX = pMsg->x;
	g_fGlobalRobotY = pMsg->y;
	g_fGlobalRobotTheta = pMsg->theta;
}

void receiveIRObstacleDetection(const ir_obstacle_detection::IRObstacleSignal::ConstPtr& pMsg)
{
    g_pIRData = pMsg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_creation");

    ros::NodeHandle n;

    g_map.create(g_fMAP_WIDTH, g_fMAP_HEIGHT, g_fCELL_SIZE);

    g_GlobalPositionEstimationSub = n.subscribe("/motion/Odometry", 1, receiveGlobalPositionEstimation);
    g_IRObstacleDetectionSub = n.subscribe("ir_obstacle_detection/IRObstacleSignal", 1, receiveIRObstacleDetection);

    g_MapPub = n.advertise<map_creation::Map>("/map_creation/Map", 1);

    ros::Rate loop_rate(100);
    ros::Duration map_update(1.0);
    ros::Time timestamp = ros::Time::now(), current_time;

    while (ros::ok())
    {
        detectObstacles();
        g_map.placeRobot(g_fGlobalRobotX, g_fGlobalRobotY, g_fGlobalRobotTheta);

        /*
         * Update map every <map_update> milliseconds.
         */
        current_time = ros::Time::now();
        if ((current_time - timestamp) > map_update)
        {
            ROS_INFO("Update map");
            timestamp = current_time;
            std::tr1::shared_ptr<map_creation::Map> pMapMsg;
            g_map.exportMap(pMapMsg);
            if (pMapMsg)
            {
                // send map to visualization node
                g_MapPub.publish(*pMapMsg.get());
            }
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
