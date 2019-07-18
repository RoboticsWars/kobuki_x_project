#pragma once
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

/**
*@brief Line Detect class contains all the functions for image procesing and direction publishing
*/
class LineDetect {
 public:
    LineDetect();
    virtual ~LineDetect();
    cv::Mat img;  /// Input image in opencv matrix format
    cv::Mat img_filt;  /// Filtered image in opencv matrix format
    int dir;  /// Direction message to be published
/**
*@brief Callback used to subscribe to the image topic from the Turtlebot and convert to opencv image format
*@param msg is the image message for ROS
*@return none
*/
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
/**
*@brief Function that applies Gaussian filter in the input image 
*@param input is the image from the turtlebot in opencv matrix format
*@return Mat of Gaussian filtered image in opencv matrix format
*/
    cv::Mat Gauss(cv::Mat input);
/**
*@brief Function to perform line detection using color thresholding,image masking and centroid detection to publish direction 
*@param input is the Filtered input image in opencv matrix format
*@return int direction which returns the direction the turtlebot should head in
*/
    int colorthresh(cv::Mat input);

 private:
    ros::NodeHandle n;
    cv::Scalar LowerYellow;
    cv::Scalar UpperYellow;
    cv::Mat img_hsv;
    cv::Mat img_mask;
    ros::Publisher dirPub{n.advertise<std_msgs::Int32>("/direction", 1)};
    ros::Subscriber sub{n.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw",1,boost::bind(&LineDetect::imageCallback,this,_1))};
    std_msgs::Int32 dir_msg;
};