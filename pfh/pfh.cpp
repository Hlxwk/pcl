#include<iostream>

#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/features/pfh.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/features/normal_3d.h>
int main(int argc,char ** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::PFHSignature125> pfh;
    pcl::PCDReader reader;
    reader.read(argv[1],*cloud);
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree1);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>());
    pfh.setSearchMethod(tree1);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125>);
    pfh.setRadiusSearch(0.05);
    pfh.compute(*pfhs);
    std::cout<<"done!\n";
    return 0;
}