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

#include <hand_detection/HandDetection.h>

/** Publisher for hand detection messages. */
ros::Publisher g_HandDetectionPub;

/** Subscriber on depth image topic. */
image_transport::Subscriber g_ImgSub;

/** Position of the detected hand (actually a mean of all possible detected candidates). */
cv::Point g_ptMean(0.0, 0.0);

/** Depth of the detected mean hand position. */
float g_fDepth = 0.0;

/** Flag for hand detection. */
bool g_bIsHandDetected = false;

/** Number of candidates that a mean is calculated from. */
size_t g_nCandidates = 1;

/** List of candidates for the estimated hand position. */
std::vector<cv::Point> g_vecHandPointCandidates;

/** Minimal depth for a detected hand [m]. */
float g_fMIN_DEPTH = 0.0;

/** Maximal depth for a detected hand [m]. */
float g_fMAX_DEPTH = 3.20;

/** Punishement for invalid cases that influences detection of hand. */
unsigned int g_nPunishement = 0;


/**
 * Receive the depth image via ros-message and perform a simple hand detection.
 * 
 * @param pImage
 *	a ros message containing the depth image
 */
void depthImage(const sensor_msgs::Image::ConstPtr & pImage)
{
    // convert ros image message to openCV image
    cv_bridge::CvImagePtr pDepthImageContainer;
    try 
    {
        pDepthImageContainer = cv_bridge::toCvCopy(pImage);
    } 
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }      

    // in the following use a pointer for short-cut access to the depth image
    cv::Mat* pDepthImage = &pDepthImageContainer->image;

    cv::GaussianBlur(*pDepthImage, *pDepthImage, cv::Size(5,5), 0, 0);

    // create a circular mask
    cv::Mat maskImage(pDepthImage->rows, pDepthImage->cols, CV_8UC1, cv::Scalar(0));
    cv::Point ptEllipseCenter(pDepthImage->cols/2, pDepthImage->rows/2);
    double fScale = 1.0; // Edit this to scale the mask
    cv::Size sizeEllipseSize(fScale*pDepthImage->rows*0.4, fScale*pDepthImage->cols*0.2);
    cv::Scalar ellipseColor(255);
    cv::ellipse(maskImage, ptEllipseCenter, sizeEllipseSize, 0., 0., 360., ellipseColor, -1);
    
    // detect hand
    cv::Point ptHand;
    cv::Point ptMean = g_ptMean;
    double fMaxValue;
    double fMinValue;
    cv::minMaxLoc(*pDepthImage, &fMinValue, &fMaxValue, &ptHand, (cv::Point*) 0, maskImage);



    // stretch pixel values to become visible on cv::imshow()
    //pDepthImage->convertTo(*pDepthImage, CV_8UC1, 255./fMaxValue);
/*
    if (g_vecHandPointCandidates.size() < g_nCandidates)
    {
        if (pDepthImage->at<float>(ptMean.x, ptMean.y) != pDepthImage->at<float>(ptMean.x, ptMean.y)
            || pDepthImage->at<float>(ptMean.x, ptMean.y) < g_fMIN_DEPTH
            || pDepthImage->at<float>(ptMean.x, ptMean.y) > g_fMAX_DEPTH)
        {
            // skip invalid points and not a number case
            ++g_nPunishement;
        }
        else    
            g_vecHandPointCandidates.push_back(ptHand);
    }
    else
    {        
        std::vector<cv::Point>::iterator itHandPoint;
        for (itHandPoint = g_vecHandPointCandidates.begin(); itHandPoint != g_vecHandPointCandidates.end(); ++itHandPoint)
        {
            ptMean += *itHandPoint;
        }
        ptMean *= 1. / (g_nCandidates + 1);

        g_ptMean = ptMean;
        g_fDepth = (pDepthImage->at<float>(g_ptMean.x, g_ptMean.y) - g_fMIN_DEPTH) / (g_fMAX_DEPTH - g_fMIN_DEPTH);

        g_vecHandPointCandidates.clear();
        g_nPunishement = 0;
    }

    if (g_nPunishement < 50){
        g_ptMean = 0.0 * g_ptMean;
    }
    g_bIsHandDetected = (g_nPunishement < 50);*/
    

    g_fDepth = (pDepthImage->at<float>(ptHand.x, ptHand.y) - g_fMIN_DEPTH) / (g_fMAX_DEPTH - g_fMIN_DEPTH);


    printf("%g\n", g_fDepth);
    g_bIsHandDetected = (g_fDepth < 0.7 && g_fDepth > 0.1);
    if (g_nPunishement > 10){
        g_bIsHandDetected = false;
    } 
    if(g_fDepth != g_fDepth || g_fDepth == 0){
        ++g_nPunishement;
    } else {
        g_nPunishement = 0;    
    }
    // convert detected point to another base coordinate system that has its origin in the image center
    cv::Point ptCenteredMean = (ptHand - cv::Point(pDepthImage->cols / 2, pDepthImage->rows / 2));
    
    // publish current status of hand detection
    hand_detection::HandDetection handDetectionMsg;
    handDetectionMsg.fX = ptCenteredMean.x * (2. / pDepthImage->cols);
    handDetectionMsg.fY = -ptCenteredMean.y * (2. / pDepthImage->rows);
    handDetectionMsg.fDepth = std::max(0.0f, g_fDepth);
    handDetectionMsg.bIsDetected = g_bIsHandDetected;
    g_HandDetectionPub.publish(handDetectionMsg);

    cv::Mat handImage = cv::Mat::zeros(pDepthImage->rows, pDepthImage->cols, CV_32FC3);
    cv::ellipse(*pDepthImage, ptHand, cv::Size(10.,10.), 0, 0, 360, cv::Scalar(0, 0, 0), 5);
    cv::imshow("Hand detection", *pDepthImage);
    cv::waitKey(3);
}


/**
 * This node detects an object, e.g. a hand. in front of the camera and publishes information about that.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hand_detection");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    g_ImgSub = it.subscribe("/camera/depth/image", 1000, depthImage);

    g_HandDetectionPub = n.advertise<hand_detection::HandDetection>("/human/hand_detection", 100);

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
