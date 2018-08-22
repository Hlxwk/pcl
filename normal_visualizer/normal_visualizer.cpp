#include<iostream>

#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<boost/thread/thread.hpp>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include<pcl-1.7/pcl/common/common_headers.h>
#include<pcl-1.7/pcl/features/normal_3d.h>
#include<pcl-1.7/pcl/console/parse.h>


int main(int argc,char ** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(argv[1],*cloud);
    std::vector<int> indices(floor(cloud->points.size()/10));
    for(size_t i=0;i<indices.size();i++)indices[i]=i;
    boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int>(indices));
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setIndices(indicesptr);
    ne.compute(*normals);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("1111"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,"original cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"original cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals,10,0.05,"normals");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;
    
}