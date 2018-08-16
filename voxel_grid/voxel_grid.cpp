#include<iostream>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/filters/voxel_grid.h>
#include<pcl-1.7/pcl/io/io.h>
#include<pcl-1.7/pcl/visualization/cloud_viewer.h>

/*
int use_data;
void viewerOneOff(pcl::visualization::PCLVisualizer viewer)
{
    viewer.setBackgroundColor(1.0,0.5,1.0);
    pcl::PointXYZ c;
    c.x=1.0;
    c.y=0;
    c.z=0;
    viewer.addSphere(c,0.25,"sphere",0);
    std::cout<<"i only run once!"<<std::endl;
}
void viewerPsycho(pcl::visualization::PCLVisualizer viewer)
{
    static unsigned count =0;
    std::stringstream ss;
    ss<<"Once per viewer loop: "<<count++;
    viewer.removeShape("text",0);
    viewer.addText(ss.str(),200,300,"text",0);
    use_data++;
}
*/
int main(int argc,char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    if(argc!=2)
    {
        std::cerr<<"please input a dir of pcd file!\n";
        exit(0);
    }
    if(pcl::io::loadPCDFile(argv[1],*cloud)==-1)
    {
        PCL_ERROR("this dir does not exit pcd file\n");
        return (-1);
    }
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f,0.01f,0.01f);
    sor.filter(*cloud_filtered);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("111"));
    int v1(0),v2(0);
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
    viewer->addText("original",10,10,"v1 text",v1);
    viewer->addPointCloud(cloud,"original cloud",v1);
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->addText("filtered",10,10,"v2 text",v2);
    viewer->addPointCloud(cloud_filtered,"filtered cloud",v2);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return (0);
}