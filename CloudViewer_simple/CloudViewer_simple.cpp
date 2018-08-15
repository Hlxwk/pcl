#include<pcl-1.7/pcl/visualization/cloud_viewer.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<iostream>
int main(int argc,char ** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(argc!=2)
    {
        using namespace std;
        cerr<<"Please input the dir of pcd_file!"<<endl;
        exit(0);
    }
    if(pcl::io::loadPCDFile(argv[1],*cloud)==-1)
    {
        PCL_ERROR("this dir does not exit %s pcd file!\n",argv[1]);
        return(-1);
    }
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while(!viewer.wasStopped())
    {

    }
    return(0);


}
