#include <iostream>
#include <fstream>

#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<pcl-1.7/pcl/io/pcd_io.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;

const float VOXEL_GRID_SIZE = 0.01;
const double radius_normal=20;
const double radius_feature=50;
const double max_sacis_iteration=1000;
const double min_correspondence_dist=0.01;
const double max_correspondence_dist=1000;

pcl::console::TicToc timecal;

void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, float gridsize){
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setLeafSize(gridsize, gridsize, gridsize);
	vox_grid.setInputCloud(cloud_in);
	vox_grid.filter(*cloud_out);
	return;
}

pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normalsPtr (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> norm_est;
    norm_est.setInputCloud(cloud);
    norm_est.setRadiusSearch(radius);
    norm_est.compute(*normalsPtr);
    return normalsPtr;

}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals,double radius)
{
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(radius);
    fpfh_est.compute(*features);
    return features;
}

Eigen::Matrix4f sac_ia_align(pcl::PointCloud<pcl::PointXYZ>::Ptr source,pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                             pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_feature,pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_feature,
                             int max_sacia_iterations,double min_correspondence_dist,double max_correspondence_dist)
{
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> sac_ia;
    Eigen::Matrix4f final_transformation;
    sac_ia.setInputSource(target);
    sac_ia.setSourceFeatures(target_feature);
    sac_ia.setInputTarget(source);
    sac_ia.setTargetFeatures(source_feature);
    sac_ia.setMaximumIterations(max_sacia_iterations);
    sac_ia.setMinSampleDistance(min_correspondence_dist);
    sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalcloud (new pcl::PointCloud<pcl::PointXYZ>);
    timecal.tic();
    sac_ia.align(*finalcloud);
    cout<<"Finished SAC_IA Initial Regisration in "<<timecal.toc()<<"ms"<<endl;
    final_transformation=sac_ia.getFinalTransformation();
    return final_transformation;

}

int main(int argc,char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(argv[1],*source)<0)
    {
        PCL_ERROR("This dir doesnot exit %s pcd file.\n",argv[1]);
        return(-1);
    }
    if(pcl::io::loadPCDFile(argv[2],*target)<0)
    {
        PCL_ERROR("This dir doesnot exit %s pcd file.\n",argv[2]);
        return(-1);
    }
    vector<int> indices1;
    vector<int> indices2;
    pcl::removeNaNFromPointCloud(*source,*source,indices1);
    pcl::removeNaNFromPointCloud(*target,*target,indices2);
//降采样
    voxelFilter(source,source,VOXEL_GRID_SIZE);  
    voxelFilter(target,target,VOXEL_GRID_SIZE);  
 
//计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr source_normal (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normal (new pcl::PointCloud<pcl::Normal>);
    source_normal=getNormals(source,radius_normal);
    target_normal=getNormals(target,radius_normal);
   
//计算FPFH特征    
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_feature (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_feature (new pcl::PointCloud<pcl::FPFHSignature33>);
    source_feature=getFeatures(source,source_normal,radius_feature);
    target_feature=getFeatures(target,target_normal,radius_feature);
  
//SAC-IA配准
    Eigen::Matrix4f init_transform;
    init_transform=sac_ia_align(source,target,source_feature,target_feature, max_sacis_iteration,min_correspondence_dist,max_correspondence_dist);
    pcl::transformPointCloud(*target,*result,init_transform);    
    cout<<init_transform<<endl;
    return 0;






}
