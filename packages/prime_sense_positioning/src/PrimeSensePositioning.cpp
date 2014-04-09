#include <stdlib.h>

#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

image_transport::Subscriber g_dImgSub;


void recieveDepthImage(const sensor_msgs::Image::ConstPtr & pImage)
{
     cv_bridge::CvImagePtr pDepthImageContainer;
    try
    {
        pDepthImageContainer = cv_bridge::toCvCopy(pImage);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::Mat* pDepthImage = &pDepthImageContainer->image;

    // TODO: Get distance to two points on the same row and calculate the distance between them
    // in order to calculate the angle theta (same distance means we're level).
    // TODO: Get distance to wall in front and compare it to the stored distance to the wall in
    // front in order to calculate how our x/y value has changed.
    // TODO: Store if we are currently moving along the x or y axis in order to know which should
    // be calculated.

    cv::imshow("Depth Image", *pDepthImage);
    cv::waitKey(3);

}




/**
 * This node calculates the x and y coordinates and the orientation theta for the robot.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "prime_sense_positioning");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    g_dImgSub = it.subscribe("/camera/depth/image", 100, recieveDepthImage);



    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
