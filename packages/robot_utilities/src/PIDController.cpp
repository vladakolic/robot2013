#include "ros/ros.h"

#include <robot_utilities/PIDController.h>

float g_fIntegratorPart = 0.0;

namespace robot_utilities
{

PIDController::PIDController()
{
    m_fIntegrator = 0.0;
    m_fDerivator = 0.0;
    ROS_WARN("Controller was reconstructed!");
}

PIDController::~PIDController()
{

}

void PIDController::reset(const float& fSetPoint, const float& fErrorMinimum, const float& fIntegratorMinimum, const float& fIntegratorMaximum)
{
    /* 
     * Detect a sign change on the reference signal. A reference of 0 is omitted since it will prevent the detection of
     * a sign change with the used method. In the case of a detected sign change the integrator part is resetted.
     */
    if (fSetPoint != 0.f) 
    {
        // if old and new value have different signs the division will be less than zero
        if ((fSetPoint / m_fLastSetPoint) < 0) 
        {
            // reset the integrator
            m_fIntegrator = 0.0f;
        }

        // only in the case of a non-zero set point the m_fLastSetPoint variable is updated
        m_fLastSetPoint = fSetPoint;
    }
    // the m_fSetPoint variable is updated in every case
    m_fSetPoint = fSetPoint;

    m_fIntegratorMinimum = fIntegratorMinimum;
    m_fIntegratorMaximum = fIntegratorMaximum;
    m_fErrorMinimum = fErrorMinimum;
}

float PIDController::update(const float& fMeasurement, const float& fDeltaT)
{
    float fPValue, fIValue, fDValue, fError;

    fError = m_fSetPoint - fMeasurement;
    fPValue = m_fKp * fError;

    m_fIntegrator += fError * fDeltaT;
    fIValue = m_fKi * m_fIntegrator;
// TODO: remove >>>
// just a debug output for being able to follow the development of the integral part.
    if (m_fKi > 0.0)
        //ROS_INFO("fIValue = %g m_fIntegrator = %g fError = %g fDeltaT = %g", fIValue,  m_fIntegrator, fError, fDeltaT);
// <<<
    if (fIValue > m_fIntegratorMaximum)
    {
        //ROS_WARN("Controller exceeded maximum in integrator part (%g > %g)", fIValue, m_fIntegratorMaximum);
        fIValue = m_fIntegratorMaximum;
    }
    if (fIValue < m_fIntegratorMinimum)
    {
        //ROS_WARN("Controller exceeded minimum in integrator part (%g < %g)", fIValue, m_fIntegratorMinimum);
        fIValue = m_fIntegratorMinimum;
    }

    fDValue = m_fKd * (fError - m_fDerivator) / fDeltaT;
    m_fDerivator = fError;

    m_fY = fPValue + fIValue + fDValue;

    // With the use of a minimum error we want to prevent the output signal from oscillating at low ranges
    if (fabs(fError) < m_fErrorMinimum)
        return 0.0;
    else
        return m_fY;
}

void PIDController::setGains(const float& fKp, const float& fKi, const float& fKd)
{
    m_fKp = fKp;
    m_fKi = fKi;
    m_fKd = fKd;
}

void PIDController::getGains(float& fKp, float& fKi, float& fKd)
{
    fKp = m_fKp;
    fKi = m_fKi;
    fKd = m_fKd;
}

} // namespace robot_utilities
