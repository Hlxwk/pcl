#include<iostream>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/point_types.h>
int main(int argc,char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(argv[1],*cloud)==-1)
    {
        PCL_ERROR("this dir does not exit %s pcd file./n",argv[1]);
        return(-1);
    }
    for(size_t i=0;i<cloud->points.size();++i)
    {
        std::cerr<<cloud->points[i].x<<" "<<cloud->points[i].y<<std::endl;
    }
    return 0;
}