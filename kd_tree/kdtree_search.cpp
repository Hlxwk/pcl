#include<pcl-1.7/pcl/point_cloud.h>
#include<pcl-1.7/pcl/kdtree/kdtree_flann.h>

#include<iostream>
#include<vector>
#include<ctime>
int main(int argc,char ** argv)
{
    srand(time(NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width=1000;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);

    for(size_t i=0;i<cloud->points.size();i++)
    {
        cloud->points[i].x=1024.0f*rand()/(RAND_MAX+1.0f);
        cloud->points[i].y=1024.0f*rand()/(RAND_MAX+1.0f);
        cloud->points[i].z=1024.0f*rand()/(RAND_MAX+1.0f);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointXYZ searchPoint;
    searchPoint.x=1024.0f*rand()/(RAND_MAX+1.0f);
    searchPoint.y=1024.0f*rand()/(RAND_MAX+1.0f);
    searchPoint.z=1024.0f*rand()/(RAND_MAX+1.0f);

    int K=10;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSqureDistance(K);

    std::cout<<"K nearest neighbor search at ("<<searchPoint.x<<" "<<searchPoint.y<<" "<<searchPoint.z<<") with K="<<K<<std::endl;
    if(kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSqureDistance)>0)
    {
        for(size_t i=0;i<pointIdxNKNSearch.size();i++)
        std::cout<<" "<<cloud->points[pointIdxNKNSearch[i]].x<<" "<<cloud->points[pointIdxNKNSearch[i]].y<<" "<<cloud->points[pointIdxNKNSearch[i]].z<<"(squre distance: "<<pointNKNSqureDistance[i]<<")"<<std::endl;

    }

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquareDistance;

    float radius=256.0f*rand()/(RAND_MAX+1.0f);
    std::cout<<"Neighbors within radius search at ("<<searchPoint.x<<" "<<searchPoint.y<<" "<<searchPoint.z<<") with radius ="<<radius<<std::endl;

    if(kdtree.radiusSearch(searchPoint,radius,pointIdxRadiusSearch,pointRadiusSquareDistance)>0)
    {
        for(size_t i=0;i<pointIdxRadiusSearch.size();i++)
        {
            std::cout<<" "<<cloud->points[pointIdxRadiusSearch[i]].x<<" "<<cloud->points[pointIdxRadiusSearch[i]].y
            <<cloud->points[pointIdxRadiusSearch[i]].z<<"(square distance: "<<pointRadiusSquareDistance[i]<<")"<<std::endl;
        }
        return 0;
    }
}