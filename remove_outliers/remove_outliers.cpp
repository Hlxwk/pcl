#include<iostream>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/filters/radius_outlier_removal.h>
#include<pcl/filters/conditional_removal.h>
#include<pcl-1.7/pcl/console/parse.h>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>
int main(int argc, char ** argv)
{
    bool model_c(0),model_r(0);
    if(pcl::console::find_argument(argc,argv,"-c")>=0)model_c=1;
    if(pcl::console::find_argument(argc,argv,"-r")>=0)model_r=1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(argv[2],*cloud);
    if(model_r)
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.1);
        outrem.setMinNeighborsInRadius(2);
        outrem.filter(*cloud_filtered);
    }
    else if(model_c)
    {
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (cloud);
        condrem.setKeepOrganized(true);
        condrem.filter (*cloud_filtered);
    }
    int v1(0),v2(0);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("111"));
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
    viewer->addPointCloud(cloud,"original cloud",v1);
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->addPointCloud(cloud,"filtered cloud",v2);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;

}