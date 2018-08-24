#include<iostream>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/features/fpfh.h>
#include<pcl-1.7/pcl/features/normal_3d.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
int main(int argc,char ** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(argv[1],*cloud);
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.setRadiusSearch(0.05);
    fpfh.compute(*fpfhs);
    std::cout<<"done\n";
    return 0;
}