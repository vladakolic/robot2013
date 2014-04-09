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


image_transport::Subscriber g_cImgSub;
image_transport::Subscriber g_dImgSub;

cv::vector<cv::Vec4i> houghLines;


void detectWall(cv::Mat& rDepthImg) {
    cv::Size size = rDepthImg.size();
    
}

/**
* Parse a given image using Canny and Hough Line Transform.
* 
@param pImage
*   A pointer to the image we wish to parse.
*/
void houghLineTransform(cv::Mat& image){

    cv::Mat dst;
    int low_threshold = 50;
    int ratio = 4;
    int kernel_size = 3;
    // Convert color image to gray scale
    cv::cvtColor( image, dst, CV_BGR2GRAY );

    cv::Canny(dst, dst, low_threshold, low_threshold*ratio, kernel_size);

    
    // Probabilistic Hough Line Transform
    //cv::vector<cv::Vec4i> lines;
    HoughLinesP(dst, houghLines, 1, CV_PI/180, 50, 50, 10 );

}

void plotHoughLines(cv::Mat& img){
    for( size_t i = 0; i < houghLines.size(); ++i)
    {
        cv::Vec4i l = houghLines[i];
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
    }
    
}

/**
 * Receive the color image via ros-message and apply Canny edgedetection
 * and Hough Line Transform.
 * 
 * @param pImage
 *	a ros message containing the color image
 */
void receiveColorImage(const sensor_msgs::Image::ConstPtr & pImage)
{
    // convert ros image message to openCV image
    cv_bridge::CvImagePtr pColorImageContainer;
    try 
    {
        pColorImageContainer = cv_bridge::toCvCopy(pImage);//, sensor_msgs::image_encodings::TYPE_16UC1);
    } 
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }      

    // in the following use a pointer for short-cut access to the depth image
    cv::Mat* pColorImage = &pColorImageContainer->image;

    houghLineTransform(*pColorImage);
    plotHoughLines(*pColorImage);
    cv::imshow("Color Image", *pColorImage);
    cv::waitKey(3);
}

/**
 * Recieve the depth image via ros-message and plot the edges calculated by
 * a Hough Line Transform (requires recieveColorImage function to be working).
 *
 * @param pImage
 *  a ros message containing the depth image
 */
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

    plotHoughLines(*pDepthImage);
    cv::imshow("Depth Image", *pDepthImage);
    cv::waitKey(3);
    

}




/**
 * This node detects an object, e.g. a hand. in front of the camera and publishes information about that.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_wall_detection");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    g_cImgSub = it.subscribe("/camera/rgb/image_color", 100, receiveColorImage);
    g_dImgSub = it.subscribe("/camera/depth_registered/image", 100, recieveDepthImage);
//    g_dImgSub = it.subscribe("/camera/depth_registered/")



    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
