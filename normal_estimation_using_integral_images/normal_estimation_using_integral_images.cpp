#include<iostream>

#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/features/normal_3d.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include<pcl-1.7/pcl/features/integral_image_normal.h>

int main(int argc,char ** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(argv[1],*cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("1111"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,"origin cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"original cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals,10,0.05,"original normals");
    viewer->initCameraParameters();
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;
}