#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include <list>

#include "ros/ros.h"
#undef INT_MAX
#include <motor_control/FloatEncoders.h>
#include <differential_drive/Encoders.h>

ros::Publisher g_EncoderPub;
ros::Subscriber g_EncoderSub;


struct CircularBuffer
{
    std::list< std::pair<float, float> > m_fifo;

    size_t nMaxSize;

    CircularBuffer() : nMaxSize(10) {}
   
    std::pair<float,float> update(const std::pair<float, float>& encoderValues)
    {
        std::pair<float, float> filteredEncoders = std::pair<float, float>(0,0);

        if (m_fifo.size() >= (nMaxSize - 1))
        {
            m_fifo.push_back(encoderValues);
            
            float w0 = 1.f / 10.f;
            float wOthers = w0;
           
            size_t counter = 0; 
            std::list< std::pair<float, float> >::iterator it;
            for (it = m_fifo.begin(); it != m_fifo.end(); ++it)
            {
                filteredEncoders.first = filteredEncoders.first + it->first;
                filteredEncoders.second = filteredEncoders.second + it->second;
                counter++;
            }
            //ROS_INFO("counter = %d", counter);
            //ROS_INFO("old: %g new: %g", (float) encoderValues.first * nMaxSize, filteredEncoders.first);
            filteredEncoders.first = ((float) filteredEncoders.first * wOthers);// + (m_fifo.back().first * w0);
            filteredEncoders.second = ((float) filteredEncoders.second * wOthers);// + (m_fifo.back().second * w0);

            m_fifo.pop_front();


        }
        else
        {
            m_fifo.push_back(encoderValues);
        }

        motor_control::FloatEncoders encoderMsg;
        //ROS_INFO("filteredEncoders = %g", filteredEncoders.first);
        encoderMsg.fDeltaEncoder1 = filteredEncoders.first;//encoderValues.first;
        encoderMsg.fDeltaEncoder2 = filteredEncoders.second;//encoderValues.second;
        g_EncoderPub.publish(encoderMsg);

        return filteredEncoders;
    }
};

CircularBuffer g_EncoderBuffer;

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

/**
 * Receive encoder values from the motors.
 *
 * @param pMsg
 *  a message that contains the encoder change in [ticks]
 */
void receiveEncoders(const differential_drive::Encoders::ConstPtr& pMsg)
{
    std::pair<float, float> encoderValues = g_EncoderBuffer.update(std::pair<float, float>(float(pMsg->delta_encoder1), float(pMsg->delta_encoder2)));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "EncoderFilter");

    ros::NodeHandle n;

    // Initialize publishers
    g_EncoderPub = n.advertise<motor_control::FloatEncoders>("/motion/FilteredEncoders", 1);

    // Initialize subscribers
    g_EncoderSub = n.subscribe("/motion/Encoders", 1, receiveEncoders);

    ros::Rate loop_rate(42);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
