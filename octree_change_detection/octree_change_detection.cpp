#include<pcl-1.7/pcl/point_cloud.h>
#include<pcl-1.7/pcl/octree/octree.h>

#include<iostream>
#include<vector>
#include<ctime>
int main(int argc,char ** argv)
{
    srand((unsigned int)time(NULL));
    float resolution=32.0f;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width=128;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);
    for(size_t i=0;i<cloud->points.size();i++)
    {
        cloud->points[i].x=64.0f*rand()/(RAND_MAX+1.0f);
        cloud->points[i].y=64.0f*rand()/(RAND_MAX+1.0f);
        cloud->points[i].z=64.0f*rand()/(RAND_MAX+1.0f);
    }
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    octree.switchBuffers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ>);
    cloudB->width=128;
    cloudB->height=1;
    cloudB->points.resize(cloudB->width*cloudB->height);
    for(size_t i=0;i<cloudB->points.size();i++)
    {
        cloudB->points[i].x=64.0f*rand()/(RAND_MAX+1.0f);
        cloudB->points[i].y=64.0f*rand()/(RAND_MAX+1.0f);
        cloudB->points[i].z=64.0f*rand()/(RAND_MAX+1.0f);
    }
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();
    std::vector<int> newPointIdxVector;
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);
    std::cout<<"output form getPointIndicesFromNewVoxeis:"<<std::endl;
    for(size_t i=0;i<newPointIdxVector.size();i++)
    std::cout<<i<<"# Index:"<<newPointIdxVector[i]<<" point:"<<cloudB->points[newPointIdxVector[i]].x
    <<" "<<cloudB->points[newPointIdxVector[i]].y<<" "<<cloudB->points[newPointIdxVector[i]].z<<std::endl;


}