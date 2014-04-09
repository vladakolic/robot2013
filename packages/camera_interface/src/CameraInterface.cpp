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
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>


image_transport::Subscriber g_RGBSub;
image_transport::Subscriber g_ImgSub;


/**
 * Receive the rgb image via ros-message.
 */
void receiveRGBImage(const sensor_msgs::Image::ConstPtr& pImage)
{
    // convert ros image message to openCV image
    cv_bridge::CvImageConstPtr pRGBImageContainer;
    try 
    {
        pRGBImageContainer = cv_bridge::toCvShare(pImage, sensor_msgs::image_encodings::TYPE_32FC3);
    } 
    catch (cv_bridge::Exception & e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }      

    // in the following use a pointer for short-cut access to the depth image
    const cv::Mat* pRGBImage = &pRGBImageContainer->image;

    cv::imshow("RGB image", *pRGBImage);
    cv::waitKey(3);
}


/**
 * Receive the depth image via ros-message.
 */
void receiveDepthImage(const sensor_msgs::Image::ConstPtr & pImage)
{
    // convert ros image message to openCV image
    cv_bridge::CvImageConstPtr pDepthImageContainer;
    try 
    {
        pDepthImageContainer = cv_bridge::toCvShare(pImage, sensor_msgs::image_encodings::TYPE_8UC1);
    } 
    catch (cv_bridge::Exception & e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }      

    // in the following use a pointer for short-cut access to the depth image
    const cv::Mat* pDepthImage = &pDepthImageContainer->image;

    cv::imshow("Depth image", *pDepthImage);
    cv::waitKey(3);
}


/**
 * This node calibrates the camera.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_interface");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    g_RGBSub = it.subscribe("/camera/rgb/image_raw", 1000, receiveRGBImage);
    g_ImgSub = it.subscribe("/camera/depth/image", 1000, receiveDepthImage);

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
