
#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

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
#include "pose_estimation.h"
#include "thread.h"

/**
 * This class uses a aruco marker detection to calculate the offset
 * of the odometry pose of the BringUp launch script which is evaluating
 * the motorticks. If an aruco marker is detected the offset will be updated
 */
class ArucoDetection: public Thread
{
public:
    /**
     * Constructor
     * @param name Name of the thread
     */
    ArucoDetection(const std::string& name);

    /**
     * Destructor
     */
    virtual ~ArucoDetection();

    /**
     * Initialize PoseEstimator
     * @param poseEstimator Reference of poseEstimator
     */
    void initialize();

    /**
     * Start thread loop
     */
    void start() override;

    /**
     * Terminate thread loop
     */
    void terminate() override;

    /**
     * Return true if thead is running
     */
    bool isRunning() const;

    /**
     * Thread loop
     *
     *
     */
    void run(void) noexcept override;

    /**
     * Execute starting thread loop
     */
    void startLoop();

private:
    /**
     * check if detected marker is valid
     * @param id Detected marker to check
     */
    bool is_correct_detected(int id);

protected:
    bool m_stop;                            /*!< Flag to stop the thread */
    std::string m_name;                     /*!< Thread name*/
    cv::VideoCapture m_in_video;            /*!< Video stream of camera */
    std::vector<int> m_availableMarkers;    /*!< Vector of valid markers */
    std::map<int, tf::Vector3> m_markerMap; /*!< Map of markers and their world coordinates */
    cv::Mat m_image;                        /*!< Camera image which was taken by camera */
    cv::Mat m_cameraMatrix; /*!< Copy of camera image in case a new image is received faster */
    cv::Mat m_distCoeffs;   /*!< Dist_coefficents for marker detection */
    tf::Quaternion m_rodriguesQuat;              /*!< Rotation Quaternion of Rodrigues method */
    cv::Ptr<cv::aruco::Dictionary> m_dictionary; /*!< Used aruco dictionary */
    float m_markerLength;                        /*!< Side length of aruco marker */
    bool m_debug;                     /*!< debug flag to get more informations during run */
    tf::Transform m_cameraToMarker;   /*!< Tf from camera to marker */
    tf::Transform m_baseLinkToCamera; /*!< Tf from base_link to camera */
    tf::Transform m_mapToMarker;      /*!< Tf from map to marker*/
    tf::Quaternion m_mapMarkerQuat;   /*!< Quaternion of detected marker in world coordinates */

};

#endif   // ARUCO_DETECTION_H
