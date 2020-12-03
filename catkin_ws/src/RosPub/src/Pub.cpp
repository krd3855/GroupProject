#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

 cv::Mat m_cameraMatrix;
 cv::Mat m_distCoeffs;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    
    //cv::waitKey(10);
    cv_bridge::CvImagePtr openCvFrame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Ptr<cv::aruco::Dictionary> m_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    cv::Mat dImg =  openCvFrame_->image;
    //cv::imshow("Boobs", dImg);
    cv::FileStorage fs("../camera_matrix51.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> m_cameraMatrix;
    fs["distortion_coefficients"] >> m_distCoeffs;
    //std::cout << m_cameraMatrix << std::endl;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(dImg, m_dictionary, corners, ids);
    if (ids.size() > 0) {
      std::cout << "Markers detected : " << ids.size() << std::endl;
       std::cout << ids[0] << std::endl;
    }
   

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
