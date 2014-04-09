#include <stdlib.h>

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/nonfree/nonfree.hpp>
#include <dirent.h>

ros::Subscriber g_cImgSub;
ros::Publisher g_detObjPub;
cv::Size g_Kernel;
std::vector<cv::Mat> g_objects;


bool checkObject(cv::Mat hsvImage, cv::Mat imgThresh, cv::Scalar lower, cv::Scalar upper) {
    cv::inRange(hsvImage, lower, upper, imgThresh); 
    ROS_INFO("sum color: %f", cv::sum(imgThresh)[0]);

    cv::medianBlur(imgThresh, imgThresh, 3);
    cv::GaussianBlur(imgThresh, imgThresh, cv::Size(3,3), 0);

    cv::morphologyEx(imgThresh, imgThresh, cv::MORPH_CLOSE, 
        cv::Mat(), cv::Point(-1,-1), 1);

    // cv::imshow("image", image);
    cv::imshow("output", imgThresh);
    cv::waitKey(3);

    //ROS_INFO("amount of white pixels yao: %f", cv::sum(imgRedThresh)[0]); 
    return (cv::sum(imgThresh)[0] > 500000);
}

bool checkIfMelon(cv::Mat hsvImage, cv::Mat imgThresh, cv::Scalar lower, cv::Scalar upper) {
    cv::Mat greenImg(hsvImage.size(), CV_8U, 1);
    cv::Scalar greenLower = cv::Scalar(35,0,0);
    cv::Scalar greenUpper = cv::Scalar(80,100,100); 

    cv::inRange(hsvImage, lower, upper, imgThresh); 
    cv::inRange(hsvImage, greenLower, greenUpper, greenImg);

    cv::addWeighted(imgThresh, 1, greenImg, 1, 1, imgThresh);

    ROS_INFO("sum color: %f", cv::sum(imgThresh)[0]);

    cv::medianBlur(imgThresh, imgThresh, 3);
    cv::GaussianBlur(imgThresh, imgThresh, cv::Size(3,3), 0);

    cv::morphologyEx(imgThresh, imgThresh, cv::MORPH_CLOSE, 
        cv::Mat(), cv::Point(-1,-1), 1);

    // cv::imshow("image", image);
    cv::imshow("output", imgThresh);
    cv::waitKey(3);

    //ROS_INFO("amount of white pixels yao: %f", cv::sum(imgRedThresh)[0]); 
    return (cv::sum(imgThresh)[0] > 500000);
}



void processColorImage(const sensor_msgs::Image::ConstPtr & pImage){
	cv_bridge::CvImagePtr pColorImageContainer;
    try 
    {
        pColorImageContainer = cv_bridge::toCvCopy(pImage);
    } 
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std_msgs::String objMsg;
    
    // in the following use a pointer for short-cut access to the color image
    cv::Mat* pColorImage = &pColorImageContainer->image;

    cv::GaussianBlur(*pColorImage, *pColorImage, g_Kernel, 0);
    cv::Mat hsvImage;
    cv::cvtColor(*pColorImage, hsvImage, CV_BGR2HSV);
    cv::Mat imgThresh(pColorImage->size(), CV_8U, 1);

    // findMatch(*pColorImage);

    cv::imshow("image", *pColorImage);

    bool detected = true;

    cv::Scalar tomLower = cv::Scalar(0,100,100);
    cv::Scalar tomUpper = cv::Scalar(10,255,255);

    cv::Scalar lemLower = cv::Scalar(20,110,110);
    cv::Scalar lemUpper = cv::Scalar(40,255,255);

    cv::Scalar carLower = cv::Scalar(5,150,100);
    cv::Scalar carUpper = cv::Scalar(19,200,255);

    cv::Scalar pumLower = cv::Scalar(90,37,55);
    cv::Scalar pumUpper = cv::Scalar(115,100,100);

    cv::Scalar pepLower = cv::Scalar(170,100,100);
    cv::Scalar pepUpper = cv::Scalar(180,255,255); 

    // Melon is hard because of many colors. Main color is not enough.
    cv::Scalar melLower = cv::Scalar(100,100,100);
    cv::Scalar melUpper = cv::Scalar(180,255,255);

    cv::Scalar tigLower = cv::Scalar(10,145,0);
    cv::Scalar tigUpper = cv::Scalar(25,255,255); 

    if(checkObject(hsvImage, imgThresh, carLower, carUpper)){
        objMsg.data = "carrot";
    }
    else if(checkObject(hsvImage, imgThresh, tomLower, tomUpper))
        objMsg.data = "tomato";
    else if(checkObject(hsvImage, imgThresh, lemLower, lemUpper))
        objMsg.data = "lemon";
    else if(checkObject(hsvImage, imgThresh, pumLower, pumUpper))
        objMsg.data = "pumpkin";
    else if(checkObject(hsvImage, imgThresh, pepLower, pepUpper))
        objMsg.data = "pepper";
    else if(checkIfMelon(hsvImage, imgThresh, melLower, melUpper))
        objMsg.data = "melon";
    else if(checkObject(hsvImage, imgThresh, tigLower, tigUpper))
        objMsg.data = "tiger";
        // Works partially. Can give false positive, especially with yellow objects.
        // Should be fine since it is checked last.
    else{
        objMsg.data = "null";
        detected = false;
    }
    
    // g_detObjPub.publish(objMsg);
    // ROS_INFO("%d", isTomato);
    if(detected){
        g_detObjPub.publish(objMsg);
        ros::Duration(5).sleep();
    }
}

/**
 * This node detects an object, e.g. a tomato. in §front of the camera and 
 * publishes the name of the object it has detected.
 */
int main(int argc, char **argv)
{
    cv::initModule_nonfree();

    ros::init(argc, argv, "object_detection");

    ros::NodeHandle n;
    
    g_cImgSub = n.subscribe("/camera/rgb/image_color", 1, processColorImage);
    g_detObjPub = n.advertise<std_msgs::String>("/image/object", 10);

    g_Kernel = cv::Size(3,3);

    // initMatchImages();

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}