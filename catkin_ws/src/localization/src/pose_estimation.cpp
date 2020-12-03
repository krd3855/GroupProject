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

#include "../include/pose_estimation.h"
#include "../include/thread.h"

PoseEstimation::PoseEstimation(const std::string& name)
    : m_stop(false)
    , m_name(name)
{
}

PoseEstimation::~PoseEstimation() {}
void PoseEstimation::start()
{
    Thread::start();
}

void PoseEstimation::terminate()
{
    m_stop = true;
}

bool PoseEstimation::isRunning() const
{
    return !m_stop;
}
void PoseEstimation::startLoop()
{
    start();
}

void PoseEstimation::updateOffset(tf::Transform newOffset)
{
    m_offset = newOffset;
}

tf::Transform PoseEstimation::getGlobalOdom()
{
    return m_global_odom;
}

void PoseEstimation::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf::Quaternion q;
    m_global_odom.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
    q.setRPY(0, 0, tf::getYaw(msg->pose.pose.orientation));
    m_global_odom.setRotation(q);

    tf::Transform new_pose;
    new_pose = m_offset * m_global_odom;
    m_br.sendTransform(tf::StampedTransform(new_pose, ros::Time::now(), "map", "base_link"));

    nav_msgs::Odometry odom   = *msg;
    odom.pose.pose.position.x = new_pose.getOrigin().getX();
    odom.pose.pose.position.y = new_pose.getOrigin().getY();

    odom.pose.pose.orientation.x = new_pose.getRotation()[0];
    odom.pose.pose.orientation.y = new_pose.getRotation()[1];
    odom.pose.pose.orientation.z = new_pose.getRotation()[2];
    odom.pose.pose.orientation.w = new_pose.getRotation()[3];
    cout <<  "This is oroentaion" << odom.pose.orientation.x << endl

    m_globalOdomPubblisher.publish(odom);
}

void PoseEstimation::initialize()
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize ros related elements
    ros::NodeHandle nh;
    cout << "somethign" << endl;
    m_globalOdomPubblisher   = nh.advertise<nav_msgs::Odometry>("global_odom", 10, false);
    m_odom_sub = nh.subscribe("/odom", 10, &PoseEstimation::odomCallback, this);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Enable debug to get more information about current marker findings
    bool debug = true;
}

void PoseEstimation::run(void) noexcept
{
    while (!m_stop)
    {
        ros::spinOnce();
    }
}
