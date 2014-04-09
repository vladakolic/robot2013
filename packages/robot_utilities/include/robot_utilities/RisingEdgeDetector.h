#pragma once

namespace robot_utilities
{

/**
 * This is the implementation of a rising edge detector on boolean signals like keyboard inputs. It is useful for the 
 * implementation of online flag setting.
 */
struct RisingEdgeDetector
{
public:
    RisingEdgeDetector()
        : m_bTmp(false)
    {
    }

    bool operator()(bool bSignal)
    {
        bool bReturn = !m_bTmp && bSignal;
        m_bTmp = bSignal;
        return bReturn;
    }
private:
    bool m_bTmp;
};

} // namespace robot_utilities
