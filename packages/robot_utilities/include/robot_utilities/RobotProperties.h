#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

namespace robot_properties
{
    const float g_fTicksPerRev = 360.0;

    /** Case 1: both wheels have the same radius. */
    const float g_fRadius = 38.9E-3;

    /** Case 1: both wheels have the same circumfence. */
    const float g_fCircumfence = 2.0 * M_PI * g_fRadius;

    const float g_fTickSize = g_fCircumfence / g_fTicksPerRev;

    /** Case 2: both wheels have different radius. */
    const float g_fRadiusLeft = 0.05;//0.0779/2;

    /** Case 2: both wheels have different circumfences. */
    const float g_fCircumfenceLeft = 2.0 * M_PI * g_fRadiusLeft;

    /** Case 2: both wheels have different radius. */
    const float g_fRadiusRight = 0.05;//0.0777/2;

    /** Case 2: both wheels have different circumfences. */
    const float g_fCircumfenceRight = 2.0 * M_PI * g_fRadiusRight;

    const float g_fTickSizeLeft = g_fCircumfenceLeft / g_fTicksPerRev;

    const float g_fTickSizeRight = g_fCircumfenceRight / g_fTicksPerRev;

    /** Displacement of the two wheels, also known as B. */
    const float g_fWheelDisplacement = 0.215;

    /* 
     * The origin of the robot coordinate system is the centerpoint of the wheel-axis.
     */
    const float g_fEDGE_TO_CENTER_DISTANCE_X = 0.08f; //E--x--E
    const float g_fEDGE_TO_CENTER_DISTANCE_Y = 0.10f;

    const float g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_X = -0.087f;
    const float g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Y =  0.073f;

    const float g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_X = -0.087f;
    const float g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Y =  -0.085f;

    const float g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_X = 0.081f;
    const float g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Y = 0.073f;

    const float g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_X = 0.081f;
    const float g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Y = -0.085f;

    const float g_fIR_LEFT_FRONT_REAR_DISPLACEMENT_Y = g_fIR_FRONT_LEFT_SENSOR_DISPLACEMENT_Y - g_fIR_REAR_LEFT_SENSOR_DISPLACEMENT_Y ;
    const float g_fIR_RIGHT_FRONT_REAR_DISPLACEMENT_Y = g_fIR_FRONT_RIGHT_SENSOR_DISPLACEMENT_Y - g_fIR_REAR_RIGHT_SENSOR_DISPLACEMENT_Y;

} // namespace robot_properties

