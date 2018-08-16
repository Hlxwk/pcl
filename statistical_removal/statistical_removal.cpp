#include<iostream>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/filters/statistical_outlier_removal.h>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>

int main(int argc,char ** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    if(argc!=2)
    {
        std::cerr<<"Please input a dir of pcd file!\n";
        exit(0);
    }
    if(pcl::io::loadPCDFile(argv[1],*cloud)==-1)
    {
        PCL_ERROR("This dir does not exit pcd file!\n");
        return -1;
    }
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    int v1(0),v2(0);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("111"));
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
    viewer->addText("original",10,10,"v1 text",v1);
    viewer->addPointCloud(cloud,"original cloud",v1);
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->addText("filtered",10,10,"v2 text",v2);
    viewer->addPointCloud(cloud,"filtered cloud",v2);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return (0);
}