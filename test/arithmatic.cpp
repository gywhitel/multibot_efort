#include <pcl/common/common_headers.h>
#include "arithmatic.h"
#include <queue>


MovingAverage::MovingAverage(int windowSize)
{
  size = windowSize;
  sum.x = 0;
  sum.y = 0;
  sum.z = 0;
  sum.normal_x = 0;
  sum.normal_y = 0;
  sum.normal_z = 0;
  weight = 0.5;
}

pcl::PointNormal MovingAverage::roll(const pcl::PointNormal& point)
{
  window.push(point);   // add one in the tail
  pcl::PointNormal smoothed;
  if (window.size() > size)
  {
    sum = sum - window.front() + point;
    window.pop();   // pop out the head
    smoothed = sum / size;
    smoothed.x = point.x;   // do not alter X and Y coordinates
    smoothed.y = point.y;
    return smoothed;
  }
  sum = sum + point;
  smoothed = sum / window.size();
  smoothed.x = point.x;   // do not alter X and Y coordinates
  smoothed.y = point.y;
  return smoothed;
};

void MovingAverage::setWeight(const double w)
{
  weight = w;
}

pcl::PointNormal MovingAverage::weighted_moving_average(const pcl::PointNormal& point)
{
  window.push(point);   // add one in the tail
  pcl::PointNormal smoothed;
  if (window.size() > size)
  {
    sum = sum - window.front();
    window.pop();   // pop out the head
    smoothed = (sum * (1-weight) + point * weight)/ size;
    sum = sum + point;
    smoothed.x = point.x;   // do not alter X and Y coordinates
    smoothed.y = point.y;
    return smoothed;
  }
  smoothed = (sum * (1-weight) + point * weight) / window.size();
  sum = sum + point;
  smoothed.x = point.x;   // do not alter X and Y coordinates
  smoothed.y = point.y;
  return smoothed;
}

void MovingAverage::MovingAverageFilter(pointVec& list)
{
  for(int i = 0; i < list.size(); i++)
  {
    list[i] = roll(list[i]);
  }
}


double norm(const pcl::PointNormal &p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


pcl::PointNormal operator+(const pcl::PointNormal& p1, const pcl::PointNormal& p2)
{
  pcl::PointNormal sum;
  sum.x = p1.x + p2.x;
  sum.y = p1.y + p2.y;
  sum.z = p1.z + p2.z;
  sum.normal_x = p1.normal_x + p2.normal_x;
  sum.normal_y = p1.normal_y + p2.normal_y;
  sum.normal_z = p1.normal_z + p2.normal_z;
  return sum;
}

pcl::PointNormal operator-(const pcl::PointNormal& p1, const pcl::PointNormal& p2)
{
  pcl::PointNormal sum;
  sum.x = p1.x - p2.x;
  sum.y = p1.y - p2.y;
  sum.z = p1.z - p2.z;
  sum.normal_x = p1.normal_x - p2.normal_x;
  sum.normal_y = p1.normal_y - p2.normal_y;
  sum.normal_z = p1.normal_z - p2.normal_z;
  return sum;
}

pcl::PointNormal operator/(const pcl::PointNormal& p, const double& num)
{
    pcl::PointNormal point = p;
    point.x /= num; 
    point.y /= num; 
    point.z /= num;
    point.normal_x /= num;
    point.normal_y /= num;
    point.normal_z /= num; 
    return point;
}

pcl::PointNormal operator*(const pcl::PointNormal& p, const double& num)
{   // scalar product
    pcl::PointNormal point = p;
    point.x *= num; 
    point.y *= num; 
    point.z *= num;
    point.normal_x *= num;
    point.normal_y *= num;
    point.normal_z *= num;
    return point;
}

/**
 * @brief transform rpy euler angles to quaternions (ZYX order)
 * @param roll rotation about X axis
 * @param pitch rotation about Y axis
 * @param yaw rotation about Z axis
 */
Eigen::Quaterniond EulerAngle_to_Quaternion(double roll, double pitch, double yaw)
{
    double cr = cos(roll/2), sr = sin(roll/2);
    double cp = cos(pitch/2), sp = sin(pitch/2);
    double cy = cos(yaw/2), sy = sin(yaw/2);
    double w = cy*cp*cr + sy*sp*sr;
    double x = cy*cp*sr - cr*sy*sp;
    double y = cy*sp*cr + sy*cp*sr;
    double z = sy*cp*cr - cy*sp*sr;
    return Eigen::Quaterniond(w, x, y, z);
}