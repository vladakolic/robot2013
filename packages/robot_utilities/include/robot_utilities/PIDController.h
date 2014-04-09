#pragma once

#include "ros/ros.h"

namespace robot_utilities 
{

/**
 * Implementation of a proportional-integral-derivative (PID) controller.
 */
class PIDController
{
public:
    PIDController();

    virtual ~PIDController();
    
    /**
     * Set the controllers set-point. Eventually define min and max values for integrator term.
     *
     * @param fSetPoint
     *  the reference signal
     * @param fErrorMinimum
     *  the minimum error. If the computed error is less than the update will return 0
     * @param fIntegratorMinimum
     *  the minimum for the integrator term
     * @param fIntegratorMaximum
     *  the maximum for the integrator term
     */
    void reset(const float& fSetPoint, const float& fErrorMinimum = 0., const float& fIntegratorMinimum = -255., const float& fIntegratorMaximum = 255.);

    /**
     * Updates the controller, given the current measurement.
     *
     * @param fMeasurement
     *  the current measurement that should be compared to the set-point
	 * @param fDeltaT
	 *	the current timestep
     * @return
     *  pid-value
     */
    float update(const float& fMeasurement, const float& fDeltaT = 0.01);

    /**
     * Write the gains.
     */
    void setGains(const float& fKp, const float& fKi, const float& fKd);

    /**
     * Read the gains.
     */
    void getGains(float& fKp, float& fKi, float& fKd);

private:
    /** These gains regulate the three different terms of the pid-controller. */
    float m_fKp, m_fKi, m_fKd;

    /** The integrator term. */
    float m_fIntegrator;

    /** The minimum of the integrator term. */
    float m_fIntegratorMinimum;

    /** The maximum of the integrator term. */
    float m_fIntegratorMaximum;

    /** The derivator term. */
    float m_fDerivator;

    /** The controller output. */
    float m_fY;

    /** The set-point. */
    float m_fSetPoint;

    /** Temporary storage of old set-point. This is used to detect a sign change that should reset the integral part. */
    float m_fLastSetPoint;

    /** A minimum for the calculated error. If it is reached the update will return 0. */
    float m_fErrorMinimum;

}; // class PIDController

} // namespace robot_utilities 
