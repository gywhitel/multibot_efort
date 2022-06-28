#ifndef PATH
#define PATH


#include <pcl/common/common_headers.h>

using PointT = pcl::PointXYZ;
using pointVec = std::vector<pcl::PointNormal>;

// Define parameters
const bool downsample = false;   // enable downsampling
const double step = 0.02;    // distance between two adjacent intersection planes
const double within = step/2;     // the threshold, less than which the points intersect with the plane = step / 2
const double pointDensity = 0.0002;   // the waypoint distance of a path
const bool transform = true;
const double LEFT_LIMIT = 0.15; 
const double RIGHT_LIMIT = -LEFT_LIMIT;
// const double forward_limit = -0.03;
const double forward_limit = -0.1;
const double backward_limit = -forward_limit;

// std::array<double,2> range(pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud);

// std::array<double,2> range(pointVec &intersectPoint);

// pcl::PointNormal average(pointVec &neighborhood);

// pointVec intersect(pcl::PointCloud<pcl::PointNormal>::ConstPtr surface, const float planeX);

// pointVec rearrange(pointVec &intersectPoint);

void vec_to_cloud(pointVec &path, pcl::PointCloud<pcl::PointNormal> &cloud);

/** @brief generate grinding path from input pointcloud 'surface', and put generated path points into pathCloud pointcloud for visualization.
 * @param surface: `pcl::PointNormal` pointcloud(xyz + normal vector). The cloud shouble be decentered so that intersection plane method can be adopted.
 * @param pathCloud: put generated path points into pcl::PointNormal pointcloud for visualization.
*/
void pathGenerationCloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr surface, pcl::PointCloud<pcl::PointNormal>::Ptr pathCloud);

// pcl::PointNormal extrapolate(pointVec &path, const int i, const double waypointY);

/**
 * @brief Read a postprocessed pointcloud and generate grind paths based on intersection plane method
 * @param data the filename of input point cloud (should be properly preprocessed)
 * @param paths a vector of polishing paths
 * @param connection a vector of connections that join paths
 */
void generate_path_from_pointcloud(std::string& data, std::vector<pointVec>& paths, std::vector<pointVec>& connection);

// pointVec pathGeneration(pcl::PointCloud<pcl::PointNormal>::ConstPtr surface);

void pathGeneration(pcl::PointCloud<pcl::PointNormal>::ConstPtr surface, std::vector<pointVec>& paths, std::vector<pointVec>& connection);

pointVec linearInterpolate(const pcl::PointNormal& start, const pcl::PointNormal& end, const double density);

#endif