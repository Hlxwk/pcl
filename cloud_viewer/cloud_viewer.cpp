#include<pcl-1.7/pcl/visualization/cloud_viewer.h>
#include<iostream>
#include<pcl-1.7/pcl/io/io.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
int user_data;
void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(1.0,0.5,1.0);
    pcl::PointXYZ o;
    o.x=0;
    o.y=0;
    o.z=0;
    viewer.addSphere(o,1,"sphere",0);
    std::cout<<"i only run once"<<std::endl;
}
void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count=0;
    std::stringstream ss;
    ss<<"Once per viewer loop: "<<count++;
    viewer.removeShape("text",0);
    viewer.addText(ss.str(),200,300,"text",0);
    user_data++;
}
int
main(int argc,char ** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(argc!=2)
    {
        std::cerr<<"Please input a dir of pcd_file!\n"<<std::endl;
        exit(0);
    }
    if(pcl::io::loadPCDFile(argv[1],*cloud)==-1)
    {
        PCL_ERROR("This dir does not pcd files\n");
        return(-1);
    }
    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(cloud);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    viewer.runOnVisualizationThread(viewerPsycho);
    while(!viewer.wasStopped())
    {
        user_data++;
    }
    std::cerr<<user_data<<std::endl;
    return 0;

}