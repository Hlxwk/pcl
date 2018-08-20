#include<iostream>

#include<pcl-1.7/pcl/console/parse.h>
#include<pcl-1.7/pcl/filters/extract_indices.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/sample_consensus/ransac.h>
#include<pcl-1.7/pcl/sample_consensus/sac_model_plane.h>
#include<pcl-1.7/pcl/sample_consensus/sac_model_sphere.h>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer>//important;
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("111"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"sample cloud");
    viewer->initCameraParameters();
    return viewer;
}
int main(int argc,char ** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(argv[2],*cloud);
    /*
    cloud->width=500;
    cloud->height=1;
    cloud->is_dense=false;
    cloud->points.resize(cloud->width*cloud->height);
    for(size_t i=0;i<cloud->points.size();i++)
    {
        if(pcl::console::find_argument(argc,argv,"-s")>=0 || pcl::console::find_argument(argc,argv,"-sf")>=0)
        {
            cloud->points[i].x=1024*rand()/(RAND_MAX+1.0f);
            cloud->points[i].y=1024*rand()/(RAND_MAX+1.0f);
            if(i%5==0)
            cloud->points[i].z=1024*rand()/(RAND_MAX+1.0f);
            else if (i%2==0)
            cloud->points[i].z= sqrt(1-(cloud->points[i].x*cloud->points[i].x)
                                      -(cloud->points[i].y*cloud->points[i].y));
            else
            cloud->points[i].z=-sqrt(1-(cloud->points[i].x*cloud->points[i].x)
                                      -(cloud->points[i].y*cloud->points[i].y));
            
        }
        else
        {
            cloud->points[i].x=1024*rand()/(RAND_MAX+1.0f);
            cloud->points[i].y=1024*rand()/(RAND_MAX+1.0f);
            if(i%2==0)
            cloud->points[i].z=1024*rand()/(RAND_MAX+1.0f);
            else
            cloud->points[i].z=-1*(cloud->points[i].x+cloud->points[i].y);
        }
    }
    */
    std::vector<int> inliers;
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    if(pcl::console::find_argument(argc,argv,"-f")>=0)
    {
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        ransac.setDistanceThreshold(0.01);
        ransac.computeModel();
        ransac.getInliers(inliers);

    }
    else if(pcl::console::find_argument(argc,argv,"-sf")>=0)
    {
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
        ransac.setDistanceThreshold(0.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud,inliers,*final);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if(pcl::console::find_argument(argc,argv,"-f")>=0||pcl::console::find_argument(argc,argv,"-sf")>=0)
    viewer=simpleVis(final);
    else
    viewer=simpleVis(cloud);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    return 0;
}