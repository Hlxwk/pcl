#include<iostream>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/filters/passthrough.h>
#include<pcl-1.7/pcl/visualization/cloud_viewer.h>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<boost/thread/thread.hpp>
#include<pcl-1.7/pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr getnormal (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> nor;
    nor.setInputCloud(cloud);
    nor.setRadiusSearch(0.05);
    nor.compute(*getnormal);
    return(getnormal);
}
int main(int argc,char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    /*
    cloud->width=5;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);
    for(size_t i=0;i<cloud->points.size();i++)
    {
        cloud->points[i].x=1024*rand()/(RAND_MAX+1.0f);
        cloud->points[i].y=1024*rand()/(RAND_MAX+1.0f);
        cloud->points[i].z=1024*rand()/(RAND_MAX+1.0f);
    }
    */
   if(argc!=2)
   {
       std::cerr<<"Please input a dir of pcd file\n";
       exit(0);
   }
   if(pcl::io::loadPCDFile(argv[1],*cloud)==-1)
   {
       PCL_ERROR("this dir does not exit pcd files\n");
       return(-1);
   }

    /*
    std::cerr<<"Cloud before filtering: "<<std::endl;
    for(size_t i=0;i<cloud->points.size();i++)
    {
        std::cerr<<" "<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<std::endl;
    }
    */
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,1.0);
    pass.filter(*cloud_filtered);
    /*
    std::cerr<<"Cloud after filtering: "<<std::endl;
    for(size_t i=0;i<cloud_filtered->points.size();i++)
    {
        std::cerr<<" "<<cloud_filtered->points[i].x<<" "
                      <<cloud_filtered->points[i].y<<" "
                      <<cloud_filtered->points[i].z<<std::endl;
    }
    */

    int v1(0);
    int v2(0);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("111"));
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    //viewer->setBackgroundColor(0,0,0,v1);
    pcl::PointCloud<pcl::Normal>::Ptr normal1 (new pcl::PointCloud<pcl::Normal>);
    normal1=getNormals(cloud);
    viewer->addText("original",10,10,"v1 text",v1);
    viewer->addPointCloud(cloud,"original cloud",v1);
    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normal1,10,0.05,"normal1");
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    //viewer->setBackgroundColor(0.3,0.3,0.3,v2);
    viewer->addText("filtered",10,10,"v2 text",v2);
    viewer->addPointCloud(cloud_filtered,"filtered cloud",v2);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;
}