#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <map>
#include <opencv2/opencv.hpp>
#include <vector>
// ROS transform
#include <math.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <time.h>
using namespace cv;
using namespace std;

#include "../include/aruco_detection.h"
#include "../include/pose_estimation.h"
#include "../include/thread.h"

ArucoDetection::ArucoDetection(const std::string& name)
    : m_stop(false)
    , m_debug(true)
    , m_markerLength(0.0)
    , m_name(name)
{
}

ArucoDetection::~ArucoDetection() {}
void ArucoDetection::start()
{
    Thread::start();
}

void ArucoDetection::terminate()
{
    m_stop = true;
}

bool ArucoDetection::isRunning() const
{
    return !m_stop;
}

void ArucoDetection::startLoop()
{
    start();
}

void ArucoDetection::initialize()
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize OpenCV

    m_in_video.open(1);
    // Definition of type of aruco markers
    m_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    // Get camera.yml
    cv::FileStorage fs("../camera_matrix51.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> m_cameraMatrix;
    fs["distortion_coefficients"] >> m_distCoeffs;

 
}

void ArucoDetection::run(void) noexcept
{
    while (m_in_video.grab() && !m_stop)
    {
        // Receive and copy image
        cout << "isRunning" << endl;
        m_in_video.retrieve(m_image);

        // detect marker and save ids and corners
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(m_image, m_dictionary, corners, ids);

        cv::Mat rot(3, 3, CV_64FC1);

        // if at least one marker detected
        if (ids.size() > 0)
        {
            cout << "i got it baby" << endl;
        }
    m_in_video.release();
}
}

