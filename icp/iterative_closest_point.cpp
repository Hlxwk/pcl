#include <iostream>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/registration/icp.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud_in);
  pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2],*cloud_out);  

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("111"));
  viewer->setBackgroundColor(0,0,0);
  viewer->addPointCloud<pcl::PointXYZ>(Final,"222");
  while(!viewer->wasStopped())
  {
    viewer->spinOnce();
  }

 return (0);
}
