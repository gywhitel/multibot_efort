#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/common/transforms.h>
#include "../include/transformation.h"
#include "../include/path_generate.h"


// TODO: Rotation vector to Euler angle

/**
 * @brief Transform a direction vector to RPY euler angle (rotate Z axis of default frame to coincide with the vector)
 * @param vec to be transformed
 */
EulerAngle vector_to_EulerAngle(Eigen::Vector3d vec)
{
    double x = vec(0), y = vec(1), z = vec(2);
    double pitch = atan2(x, z);
    double roll = atan2(-y, sqrt(x*x + z*z));
    return EulerAngle(roll, pitch, 0);
}

/**
 * @brief transform rpy euler angles to quaternions (ZYX order)
 * @param rpy EulerAngle struct
 */
Eigen::Quaterniond EulerAngle_to_Quaternion(EulerAngle rpy)
{
    double cr = cos(rpy.roll/2), sr = sin(rpy.roll/2);
    double cp = cos(rpy.pitch/2), sp = sin(rpy.pitch/2);
    double cy = cos(rpy.yaw/2), sy = sin(rpy.yaw/2);
    double w = cy*cp*cr + sy*sp*sr;
    double x = cy*cp*sr - cr*sy*sp;
    double y = cy*sp*cr + sy*cp*sr;
    double z = sy*cp*cr - cy*sp*sr;
    return Eigen::Quaterniond(w, x, y, z);
}

/**
 * @brief transform pcl::PointNormal point to geometry_msgs::Pose message 
 * @param pn pcl::PointNormal x,y,z with normal vector (nx,ny,nz)
 * @return geometry_msgs::Pose = position (x,y,z) + orientation (w,x,y,z)
 */
geometry_msgs::Pose PointNormal_to_Pose(pcl::PointNormal pn)
{
    geometry_msgs::Pose pose;
    pose.position.x = pn.x;
    pose.position.y = pn.y;
    pose.position.z = pn.z;
    // oppposite to normal vector
    Eigen::Vector3d normal(-pn.normal_x, -pn.normal_y, -pn.normal_z);
    // Eigen::Vector3d normal(-pn.normal_x, -pn.normal_y, -pn.normal_z);
    EulerAngle rpy = vector_to_EulerAngle(normal);
    Eigen::Quaterniond quaternion = EulerAngle_to_Quaternion(rpy);
    // Orientation.X = qz because of UR ee_link frame assigning X as TCP Z axis
    pose.orientation.w = quaternion.w();
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    return pose;
}

/**
 * @brief transform pcl::PointNormal point to geometry_msgs::Twist for communication with real UR 
 * @param pn pcl::PointNormal x,y,z with normal vector (nx,ny,nz)
 * @return geometry_msgs::Twist = position (x,y,z) + orientation (r,p,y)
 */
geometry_msgs::Twist PointNormal_to_Euler(pcl::PointNormal pn)
{
    geometry_msgs::Twist pose;
    pose.linear.x = pn.x;
    pose.linear.y = pn.y;
    pose.linear.z = pn.z;
    Eigen::Vector3d normal(-pn.normal_x, -pn.normal_y, -pn.normal_z);
    EulerAngle rpy = vector_to_EulerAngle(normal);
    pose.angular.x = rpy.roll;
    pose.angular.y = rpy.pitch;
    pose.angular.z = rpy.yaw;
    return pose;
}

/** @brief transform 3x1 vector into RPY euler angle (XYZ)
*   @param 
*   @return 
*/
EulerAngle vector3_to_rpy(Eigen::Vector3d vec)
{
  return EulerAngle(vec(0), vec(1), vec(2));
}

/**
 * @brief point cloud transformation
 * @param cloudIn input pointcloud ptr
 * @param cloudOut output pointcloud ptr
*/
/*
void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut)
{
    using namespace Eigen;

    Eigen::Isometry3d transform_base_TCP = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond orientation = EulerAngle_to_Quaternion(vector3_to_rpy(TCP_orientation));
    transform_base_TCP.translate(TCP_position);  // translate from base frame to TCP frame
    transform_base_TCP.rotate(orientation);

    Eigen::Isometry3d transform_TCP_camera = Eigen::Isometry3d::Identity();
    transform_TCP_camera.translate(-camera_TCP);   // translate from camera frame to TCP frame
    
    Eigen::Isometry3d trans_base_camera;
    trans_base_camera.matrix() = transform_base_TCP.matrix() * transform_TCP_camera.matrix();

    pcl::transformPointCloud(*cloudOut, *cloudOut, trans_base_camera.matrix());

}
*/
/**
 * @brief transform pcl::PointNormal using 4x4 transformation matrix12
 * @param 
 * @return 
*/
void transformPointNormal(pcl::PointNormal& pn, const Eigen::Isometry3d& transformation)
{
  using namespace Eigen;
  Vector3d position = Vector3d(pn.x, pn.y, pn.z);
  Vector3d normal = Vector3d(pn.normal_x, pn.normal_y, pn.normal_z);
  position = transformation * position;
  normal = transformation.rotation() * normal;
  // pcl::PointNormal transPN;
  pn.x = position(0);
  pn.y = position(1);
  pn.z = position(2);
  pn.normal_x = normal(0);
  pn.normal_y = normal(1);
  pn.normal_z = normal(2);
}

/**
 * @brief transform each point (pcl::PointNormal) in vector<vector<pcl::PointNormal>>
 * @param 
 * @return 
*/
void transformPointVec(std::vector<pointVec>& vec, Eigen::Isometry3d& transformation)
{
  int i, j;
  for (i = 0; i < vec.size(); i++)
  {
    for (j =0; j < vec[i].size(); j++)
    {
      transformPointNormal(vec[i][j], transformation);
    }
  }
}

/*
* @brief normal vector (x,y,z) -> (-x,y,z)
* @param
* @return
*/
// geometry_msgs::Pose normal_mirror_YZ(geometry_msgs::Pose )