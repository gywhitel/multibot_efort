#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include "path_generate.h"
// #include "../include/transformation.h"
#include "arithmatic.h"

#define MAIN 0

using PointT = pcl::PointXYZ;
using pointVec = std::vector<pcl::PointNormal>;


// compute the distribution of the pointcloud in X axis. To make the function simple, only the case of x-axis is considered. Rotate the pointcloud if you want to get the range on other directions.
std::array<double,2> range(pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud)
{
    double min=10, max=-10;

    for (auto &point: *cloud)
    {
        if (min > point.x)
            {min = point.x;}
        if (max < point.x)
            {max = point.x;}
    }    
    if (min < RIGHT_LIMIT)
        min = RIGHT_LIMIT;
    if (max > LEFT_LIMIT)
        max = LEFT_LIMIT;
    std::array<double,2> extremum{min, max};
    return extremum;
}

// compute the distribution of the point cloud in Y axis. 
std::array<double,2> range(pointVec &intersectPoint)
{
    double min=10, max=-10;

    for (auto point: intersectPoint)
    {
        if (min > point.y)
            {min = point.y;}
        if (max < point.y)
            {max = point.y;}
    }    
    if (min < forward_limit)
        min = forward_limit;
    if (max > backward_limit)
        max = backward_limit;
    std::array<double,2> extremum{min, max};
    return extremum;
}


/** @brief extrapolate the missing point based on previous points
*   @param path a list of waypoints
*   @param i the index of the waypoint to be extrapolated
*   @param waypointY the y coordinate of the waypoint
*   @return 
*/
pcl::PointNormal extrapolate(pointVec &path, const double Y)
{   
    int i = path.size();
    assert(i >= 1);
    pcl::PointNormal waypoint;
    if (i == 1)     // the missing point has only one previous point -- min
    {   // return x,z of path[0] (path[0].x, waypointY[i], point[0].z)
        waypoint = path[0];
        waypoint.y = Y;
        return waypoint;
    }
    if (i >= 2)     // has two previous points, linear extrapolate
    {
        double z2 = path[i-1].z, y2 = path[i-1].y;
        double z1 = path[i-2].z, y1 = path[i-0].y;
        double K = (z2 - z1) / (y2 - y1);
        double B = z2 - K * y2;
        waypoint = path[i-1];
        waypoint.z = K * Y + B;
        return waypoint;
    }
    // if (i > 2)  // has more than two previous points. Use polynomial extrapolation
    // { // TODO

    // }
}

// average a list of pcl::PointNormal
pcl::PointNormal average(pointVec &neighborhood)
{
    pcl::PointNormal sum;
    sum.x = 0; sum.y = 0; sum.z = 0;
    sum.normal_x = 0; sum.normal_y = 0; sum.normal_z = 0;
    int num = 0;
    for (auto point: neighborhood)
    {
        sum = sum + point;
        num++;
    }
    sum = sum / double(num);
    return sum;
}

void vec_to_cloud(pointVec &path, pcl::PointCloud<pcl::PointNormal> &cloud)
{
    for (auto point: path)
    {
        cloud.push_back(point);
    }
}

/**
 * @brief  interpolate between two points
 * @param start interpolation starts from here
 * @param end interpolation ends here
 * @param density interpolation density. 0.005 m in default (interpolate a viapoint every 5mm)
 * @return interpolated via-points
*/
pointVec linearInterpolate(const pcl::PointNormal& start, const pcl::PointNormal& end, const double density)
{
    pcl::PointNormal vec = end - start;   // start pose - end pose. This is a screw (p, n), and each part should be computed seperately
    // normalize the screw
    double transDis = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    
    pcl::PointNormal normVec = vec / transDis;

    int numPoints = transDis / density;
    pointVec viapoints;
    pcl::PointNormal viaP;
    for (int i = 1; i < numPoints; i++)
    {
        // viaP = start + vec * double(i) * density / transDis;    // not work
        viaP = start + normVec * i * density;
        viapoints.push_back(viaP);
    }
    assert (viapoints.size() != 0);
    return viapoints;
}

// Intersect the surface pointcloud with a plane perpendicular to X-axis
pointVec intersect(pcl::PointCloud<pcl::PointNormal>::ConstPtr surface, const float planeX)
{
    pointVec intersectPoint;
    pcl::PointNormal p_bar;
    for (auto &point: *surface)
    {
        if (abs(point.x - planeX) <= within)
        {
            p_bar = point;
            p_bar.x = planeX;
            intersectPoint.push_back(p_bar);
        }
    }
    return intersectPoint;
}

// average the path points, and sort them  in y ascending order
/** @brief resample the points to get evenly distributed waypoints from point cloud in which points are unevenly distributed and noisy. More specifically, sort all the points in point-band in y-ascending order, and resample a point in every 'pointDensity' meter. Resampling is done by averaging all neighboring points near the resample point.
 * @param intersectPoint: 'pointVec' Intersected point-band
 * @return  A path (vector of waypoints) generated from point-band
*/
pointVec rearrange(pointVec &intersectPoint)
{
    auto yExtrem = range(intersectPoint);   // return points with min(y) and max(y)
    double min = yExtrem[0] + 2 * pointDensity, max = yExtrem[1] - 2 * pointDensity;

    std::vector<double> waypointY;   // resample waypoints
    waypointY.push_back(min);

    pointVec singlePath;
    pcl::PointNormal waypoint;
    // average neighbor points of a resample point 
    // for (int i = 0; i < waypointY.size(); i++)
    for (double Y = min; Y <= max; Y += pointDensity)
    {
        pointVec neighborhood;
        for (auto point: intersectPoint)
        {
            if (abs(point.y - Y) <= 0.001)    // get neighbor points
            {
                neighborhood.push_back(point);
            }
        }
        // if ((neighborhood.size() == 0) && (i == 0))
        //     continue;
        // if neighborhood is null
        if (neighborhood.size() == 0)   
        {
            waypoint = extrapolate(singlePath, Y);
        }
        if (neighborhood.size() > 0)
        {   // if the waypoint has neighbor points, average their depth values
        // A (0,0,0) point could be generated if the neighborhood is empty
            waypoint = average(neighborhood);
            waypoint.y = Y;
            singlePath.push_back(waypoint);
        }
    }
    return singlePath;
}

void pathGenerationCloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr surface, pcl::PointCloud<pcl::PointNormal>::Ptr pathCloud)
{
    std::vector<pointVec> paths;
    auto extrem = range(surface);
    double max = extrem[1], min = extrem[0]; 
    double length = max - min;
    int numSection = length / step;
    int i;
    double planeX = min;
    pointVec intersectPoint, aPath;
    for (i = 0; i < numSection+1; i++)
    {
        intersectPoint = intersect(surface, planeX);
        aPath = rearrange(intersectPoint);
        MovingAverage moving_average_filter(15);    // moving average filter
        moving_average_filter.MovingAverageFilter(aPath);
        vec_to_cloud(aPath, *pathCloud);
        if (i % 2 == 1)
        { // reserve the path
            pointVec reverse_path;
            for (auto riter = aPath.rbegin(); riter != aPath.rend(); riter++)
            {
                reverse_path.push_back(*riter);
            }
            aPath = reverse_path;
        }
        paths.push_back(aPath);
        planeX += step;
    }
    pointVec viapoints;
    for (i = 0; i < numSection; i++)
    {  // connect between two parallel paths
        int max = paths[i].size() - 1;
        viapoints = linearInterpolate(paths[i][max], paths[i+1][0], pointDensity);
        vec_to_cloud(viapoints, *pathCloud);
    }
}


void generate_path_from_pointcloud(std::string& data, std::vector<pointVec>& paths, std::vector<pointVec>& connection)
{
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PCDReader reader;
    assert (reader.read(data, *cloud) == 0);
    if (downsample)
    {
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(0.001f, 0.001f, 0.001f);
        voxel.filter(*cloud_filtered);
        *cloud = *cloud_filtered;
    }
    pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setViewPoint(0.0, 0.0, 1);  // setting viewport would change the direction of normals but also cause incorrect estimation
    normalEstimator.setRadiusSearch(0.01);
    normalEstimator.compute(*normal);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointNormal>);

    pcl::concatenateFields(*cloud, *normal, *cloud_normal);
    pathGeneration(cloud_normal, paths, connection);   
}

void pathGeneration(pcl::PointCloud<pcl::PointNormal>::ConstPtr surface, std::vector<pointVec>& paths, std::vector<pointVec>& connection)
{
    auto extrem = range(surface);
    double max = extrem[1], min = extrem[0]; 
    double length = max - min;
    int numSection = length / step;     // the number of long paths
    double planeX = min;                 // intersection plane starts from x=min
    int i;
    for (int i = 0; i < numSection+1; i++)
    {
        pointVec intersectPoint = intersect(surface, planeX);   // get intersected point-band
        pointVec aPath = rearrange(intersectPoint);     // sort the points in point-band
       
        MovingAverage moving_average_filter(15);    // moving average filter
        moving_average_filter.MovingAverageFilter(aPath);

        if (i % 2 == 1)
        { // reserve the path
        // Sort the viapoints in order, put paths and connection in a single container
        // The generated grinding path is in 'S' shape.
        // To make the trajectory continuous, the NO.odd path should be reversed, moving in y descending order.
            pointVec reverse_path;
            for (auto riter = aPath.rbegin(); riter != aPath.rend(); riter++)
            {
                reverse_path.push_back(*riter);
            }
            aPath = reverse_path;
        }
        paths.push_back(aPath);
        planeX += step;
    }
    pointVec viapoints;
    for (i = 0; i < numSection; i++)
    {  // connect between two parallel paths
        int max = paths[i].size() - 1;
        viapoints = linearInterpolate(paths[i][max], paths[i+1][0], pointDensity);
        connection.push_back(viapoints);
    }
}

#if MAIN
pcl::visualization::PCLVisualizer::Ptr pathView(pcl::PointCloud<PointT>::ConstPtr cloud, pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_normal)
{
  using namespace pcl::visualization;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  viewer->addPointCloud<PointT>(cloud, "cloud");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 0,0,0, "cloud");
  
  viewer->addPointCloud<pcl::PointNormal> (cloud_normal, "points");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 1,0,0, "points");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 3, "points");

  viewer->addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(cloud_normal, cloud_normal, 10, 0.01, "normals");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 0,1,0, "normals");
  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();
  return (viewer);
}


int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
    pcl::PCDReader reader;
    assert(reader.read(argv[1], *cloud) == 0);

    pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setViewPoint(0.0, 0.0, 0.5);  // setting viewport would change the direction of normals but also cause incorrect estimation
    normalEstimator.setRadiusSearch(0.005);
    normalEstimator.compute(*normal);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointNormal>());

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_path (new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud, *normal, *cloud_normal);
    pathGenerationCloud(cloud_normal, cloud_path);
    
    auto viewer = pathView(cloud, cloud_path);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;
}

#endif