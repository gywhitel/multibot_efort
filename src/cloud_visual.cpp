#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "transformation.h"
#include <pcl/filters/voxel_grid.h>
using PointT = pcl::PointXYZ;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pointcloud_publisher");
  ros::NodeHandle nodehandle;
  ros::Publisher pub = nodehandle.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PCDReader reader;
  assert (reader.read(argv[1], *cloud) == 0);
  cloud->header.frame_id = "pointcloud";
  std::cout<<"Point size: "<<cloud->size();
  pcl::VoxelGrid<PointT> downsampler;
  downsampler.setInputCloud(cloud);
  downsampler.setLeafSize(0.001,0.001,0.001);
  downsampler.filter(*cloud);
  std::cout<<"-> "<<cloud->size();

  // tf2::Quaternion q;
  // q.setEuler(0, M_PI, M_PI/2);  // YXZ order
  // static tf2_ros::TransformBroadcaster broadcaster;
  // geometry_msgs::TransformStamped transform;
  // transform.header.frame_id = "mid_base_link";
  // transform.child_frame_id = "pointcloud";
  // transform.transform.translation.x = 1.6;
  // transform.transform.translation.y = 0;
  // transform.transform.translation.z = 1.0;
  // transform.transform.rotation.w = q.w();
  // transform.transform.rotation.x = q.x();
  // transform.transform.rotation.y = q.y();
  // transform.transform.rotation.z = q.z();
  
  while (ros::ok())
  {
    // transform.header.stamp = ros::Time::now();
    // transform.transform.translation.x -= 0.01;
    // broadcaster.sendTransform(transform);
    pub.publish(cloud);
    ros::Duration(0.1).sleep();
  }
  
  return 0;
}