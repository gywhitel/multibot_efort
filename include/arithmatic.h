#ifndef ARITHMATIC
#define ARITHMATIC

#include <pcl-1.8/pcl/common/common_headers.h>

using PointT = pcl::PointXYZ;
using pointVec = std::vector<pcl::PointNormal>;

double norm(const pcl::PointNormal &p);

pcl::PointNormal operator+(const pcl::PointNormal& p1, const pcl::PointNormal& p2);

pcl::PointNormal operator-(const pcl::PointNormal& p1, const pcl::PointNormal& p2);

pcl::PointNormal operator/(const pcl::PointNormal& p, const double& num);

pcl::PointNormal operator*(const pcl::PointNormal& p, const double& num);

// pcl::PointNormal interpolateAdd(const pcl::PointNormal& p, const pcl::PointNormal& vec);

Eigen::Quaterniond EulerAngle_to_Quaternion(double roll, double pitch, double yaw);


class MovingAverage
{
  int size = 0;
  pcl::PointNormal sum;
  std::queue<pcl::PointNormal> window;
  double weight;

  public:
    MovingAverage(int windowSize);

    /**
     * @brief Apply moving average filter to next point
     * @param point a point with (x,y,z) and (nx, ny, nz)
     * @return smoothed point by moving average filter, but the X and Y of the original point remain
    */
    pcl::PointNormal roll(const pcl::PointNormal& point);

    pcl::PointNormal weighted_moving_average(const pcl::PointNormal& point);
    
    void setWeight(const double w);

    void MovingAverageFilter(pointVec& list);


};

#endif