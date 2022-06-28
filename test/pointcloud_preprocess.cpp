#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/console/parse.h>

#define DOWNSAMPLE 1
#define DENOISE 1
#define DEPROJECT 1

using pointT = pcl::PointXYZ;
using std::cout;
using std::endl;

pcl::visualization::PCLVisualizer::Ptr simpleVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (0.2);
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr doubleViewPorts(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addCoordinateSystem (0.2,"original", v1);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1, "cloud1", v1);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud1");
//    ruler
    pointT a, b, c, d;
    a.x = 0.5, a.y = 0, a.z = 1;
    b.x = 0, b.y = 0, b.z = 1;
    c.x = 0, c.y = 0, c.z = 0.5;
    d.x = 0.5, d.y = 0, d.z = 0.5;
    viewer->addLine(a,b,"line1", v1);
    viewer->addLine(c,d, "line2", v1);
    using namespace pcl::visualization;
    {
        viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0, "line1");
        viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0, "line2");
        
    } // namespace pcl::visualization;

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0, 0, 0, v2);
  viewer->addCoordinateSystem (0.2, "after",v2);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "cloud2", v2);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 1, 1, "cloud2");

    return viewer;
}

int main(int argc, char** argv)
{
    assert (argc > 1);
    pcl::PointCloud<pointT>::Ptr cloud (new pcl::PointCloud<pointT>());
    pcl::PointCloud<pointT>::Ptr cloud_after (new pcl::PointCloud<pointT>());
    pcl::PCDReader reader;
    assert(reader.read(argv[1], *cloud) == 0);
    
    #if DOWNSAMPLE  // downsampling
    pcl::VoxelGrid<pointT> downsampler;
    downsampler.setInputCloud(cloud);
    downsampler.setLeafSize(0.0008f, 0.0008f, 0.0008f);
    downsampler.filter(*cloud_after);
    cout<<cloud->size()<<" points in original point cloud\n";
    cout<<cloud_after->size()<<" points after downsampling\n";
    #endif

    #if DENOISE // denoise
    int meanK = 20;
    if (pcl::console::parse(argc, argv, "-k", meanK) >= 0)
        cout<<"Set mean K: "<<meanK<<endl;
    pcl::StatisticalOutlierRemoval<pointT> sor;
    sor.setInputCloud(cloud_after);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_after);

    // double radius = 0.1;
    // int neighbor_number = 20;
    // if (pcl::console::parse(argc, argv, "-r", radius) >= 0)
    //     cout<<"Set search radius: "<<radius<<endl;
    // if (pcl::console::parse(argc, argv, "-n", neighbor_number) >= 0)
    //     cout<<"Set number of neighbors in searching radius: "<<neighbor_number<<endl;
    // pcl::RadiusOutlierRemoval<pointT> outrem;
    // outrem.setInputCloud(cloud_after);
    // outrem.setRadiusSearch(radius);
    // outrem.setMinNeighborsInRadius(neighbor_number);
    // // outrem.setKeepOrganized()
    // outrem.filter(*cloud_after);
    #endif

    // deproject
    pcl::PCA<pointT> pca;
    pca.setInputCloud(cloud_after);
    pca.project(*cloud_after, *cloud_after);
    // dual port view
    // auto viewer = doubleViewPorts(cloud, cloud_after);
    auto viewer = simpleVisual(cloud_after);
    while (!viewer->wasStopped())
        viewer->spinOnce();
    cout<<"Do you want to save the point cloud ? (Y / N)\n";
    std::string answer;
    cin>>answer;
    if (answer == "Y")
    {
        cout<<"Specify the filename: ";
        cin>>answer;
        pcl::PCDWriter writer;
        writer.write(answer, *cloud_after);
        // writer.write("/home/yinghao/catkin_ws/src/multibot_efort/data/steel_preprocess.pcd", *cloud_after);

    }

    return 0;
}