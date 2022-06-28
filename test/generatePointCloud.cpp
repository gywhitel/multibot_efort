#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

using PointT = pcl::PointXYZ;

pcl::visualization::PCLVisualizer::Ptr simpleVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int main(int argc, char** argv)
{
    // generate a cylinder point cloud
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    double x, y, z;
    PointT point;
    double length = 0.3, width = 0.2, r = 0.5;

    using namespace std;
    pcl::console::parse(argc, argv, "-x", length);
    pcl::console::parse(argc, argv, "-y", width);
    pcl::console::parse(argc, argv, "-r", r);
    
    std::cout<<"Length: "<<length;
    cout<<", width: "<<width;
    cout<<" radius: "<<r<<endl;

    for (x = -length; x <= length; x += 0.001)
    {
        for (y = -width; y <= width; y += 0.001)
        {
            z = sqrt(r*r - y*y);
            point.x = x;
            point.y = y;
            point.z = z - 0.8*r;
            cloud->push_back(point);
        }
    }
    auto viewer = simpleVisual(cloud);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    string answer;
    cout<<"Save the point cloud ? (Y/N):\n";
    cin>>answer;
    if (answer == "Y")
    {
        cout<<"Name the pcd file (with .pcd): ";
        cin>>answer;
        pcl::PCDWriter writer;
        writer.write(answer, *cloud);
    }
    return 0;
}