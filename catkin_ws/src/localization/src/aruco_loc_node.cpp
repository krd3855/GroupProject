
#include <ros/ros.h>

#include "../include/aruco_detection.h"
#include "../include/pose_estimation.h"
#include "../include/thread.h"

int main(int argc, char** argv)
{
    //! Initialize ros node
    ros::init(argc, argv, "aruco_node");

    //! Initialize poseEstimator
//    PoseEstimation* poseEstimator = new PoseEstimation("pose");
  //  poseEstimator->initialize();
    //poseEstimator->startLoop();

    //! Initialize arucoDetector
    ArucoDetection* arucoDetector = new ArucoDetection("aruco");
    arucoDetector->initialize();
    arucoDetector->startLoop();

    //! Wait till both threads have been terminated
    arucoDetector->join();

};
