
#ifndef SRC_POSE_ESTIMATION_H
#define SRC_POSE_ESTIMATION_H

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "thread.h"
// ROS transform
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>


/**
 * This class uses a callback which is called by BringUp launch script on the turtlebot. 
 * If a new odomentry tf is received, the offset of the ArucoDetection is added and 
 * published for the navigation stack. 
 */
class PoseEstimation: public Thread
{
public:

    /**
    * Constructor
    * @param name Name of the thread
    */
    PoseEstimation(const std::string &name);

    /**
    * Destructor
    */
    virtual ~PoseEstimation();

    /**
    * Initialize PoseEstimator
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
    */
    void run(void) noexcept override;

    /**
    * Execute starting thread loop
    */
    void startLoop();

    /**
     * Return current odometry tf for calculation
     * the offset in ArucoDetection
     */
    tf::Transform getGlobalOdom();

    /**
     * Update the offset which was calculated by
     * ArucoDetection
     * @param newOffset Calculated Offset which will used now 
     * in the posestimation
     */
    void updateOffset(tf::Transform newOffset);

    /**
     * Callback which is called by a new Odometry message from
     * BringUp launch script on the turtlebot
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

protected:

    bool m_stop;                            /*!< Flag to stop the thread */
    std::string m_name;                     /*!< Thread name*/
    tf::Transform m_global_odom;            /*!< Tf of the received odemetry values from BringUp launch script */
    tf::TransformBroadcaster m_br;          /*!< Tf broadcaster to publish odometry msg with added offset */
    tf::Transform m_offset;                 /*!< Calculated offset from ArucoDetection */
    ros::Publisher m_globalOdomPubblisher;
    ros::Subscriber m_odom_sub;  /*!< Publisher for new odometry TF */
};

#endif   // SRC_POSE_ESTIMATION_H
