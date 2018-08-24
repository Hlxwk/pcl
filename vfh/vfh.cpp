#include<iostream>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/features/normal_3d.h>
#include<pcl-1.7/pcl/features/vfh.h>
int main(int argc,char ** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308>);
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    pcl::VFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::VFHSignature308> vfh;
    pcl::io::loadPCDFile(argv[1],*cloud);
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree1);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);
    vfh.setInputCloud(cloud);
    vfh.setInputNormals(normals);
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    vfh.compute(*vfhs);
    return 0;
}