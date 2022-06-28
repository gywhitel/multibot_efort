#ifndef TRANSFORMATION
#define TRANSFORMATION

#include <pcl-1.8/pcl/point_types.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

using PointT = pcl::PointXYZ;
using pointVec = std::vector<pcl::PointNormal>;

/** @brief RPY Euler angle (current frame)  \n 
 * Rotation order: Yaw(Z) -> Pitch(Y) -> Roll(X)
 */
struct EulerAngle
{
    /* data */
    double roll;
    double pitch;
    double yaw;

    // Constructors
    EulerAngle(): roll(), pitch(), yaw() {}
    EulerAngle(double r=0, double p=0, double y=0): roll(r), pitch(p), yaw(y) {}
};


geometry_msgs::Pose PointNormal_to_Pose(pcl::PointNormal pn);

geometry_msgs::Twist PointNormal_to_Euler(pcl::PointNormal pn);

void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut);

void transformPointVec(std::vector<pointVec>& vec, Eigen::Isometry3d& transformation);

Eigen::Quaterniond EulerAngle_to_Quaternion(EulerAngle rpy);

EulerAngle vector3_to_rpy(Eigen::Vector3d vec);

#endif