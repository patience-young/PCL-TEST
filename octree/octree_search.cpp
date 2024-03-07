#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <chrono>
#include <thread>

pcl::PointXYZRGB RandSearchPoint()
{
    pcl::PointXYZRGB searchPoint;

    searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.r = 255; // 将搜索点设置为红色
    searchPoint.g = 0;
    searchPoint.b = 0;

    return searchPoint;
}

int main ()
{
    srand ((unsigned int) time (NULL));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Generate pointcloud data
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size (); ++i)
    {
        (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
        (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
        (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);

        // 点云设置为白色
        (*cloud)[i].r = 255;
        (*cloud)[i].g = 255;
        (*cloud)[i].b = 255;
    }

    float resolution = 256.0f;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree (resolution);

    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();

    pcl::PointXYZRGB voxelSearchPoint = RandSearchPoint();

    // Neighbors within voxel search

    std::vector<int> pointIdxVec;

    if (octree.voxelSearch (voxelSearchPoint, pointIdxVec))
    {
        std::cout << "Neighbors within voxel search at (" << voxelSearchPoint.x 
            << " " << voxelSearchPoint.y 
            << " " << voxelSearchPoint.z << ")" 
            << std::endl;
                    
        for (std::size_t i = 0; i < pointIdxVec.size (); ++i) {
            std::cout << "    " << (*cloud)[pointIdxVec[i]].x 
                << " " << (*cloud)[pointIdxVec[i]].y 
                << " " << (*cloud)[pointIdxVec[i]].z << std::endl;
            
            // 设为蓝色
            (*cloud)[pointIdxVec[i]].r = 0;
            (*cloud)[pointIdxVec[i]].g = 0;
            (*cloud)[pointIdxVec[i]].b = 255;
        }
    }

    // K nearest neighbor search

    int K = 10;

    pcl::PointXYZRGB nearestKSearchPoint = RandSearchPoint();

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    std::cout << "K nearest neighbor search at (" << nearestKSearchPoint.x 
            << " " << nearestKSearchPoint.y 
            << " " << nearestKSearchPoint.z
            << ") with K=" << K << std::endl;

    if (octree.nearestKSearch (nearestKSearchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i) {
            std::cout << "    "  <<   (*cloud)[ pointIdxNKNSearch[i] ].x 
                    << " " << (*cloud)[ pointIdxNKNSearch[i] ].y 
                    << " " << (*cloud)[ pointIdxNKNSearch[i] ].z 
                    << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
            
            // 设为绿色
            (*cloud)[ pointIdxNKNSearch[i] ].r = 0;
            (*cloud)[ pointIdxNKNSearch[i] ].g = 255;
            (*cloud)[ pointIdxNKNSearch[i] ].b = 0;
        }
    }

    // Neighbors within radius search

    pcl::PointXYZRGB radiusSearchPoint = RandSearchPoint();

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 512.0f * rand () / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << radiusSearchPoint.x 
            << " " << radiusSearchPoint.y 
            << " " << radiusSearchPoint.z
            << ") with radius=" << radius << std::endl;


    if (octree.radiusSearch (radiusSearchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
            std::cout << "    "  <<   (*cloud)[ pointIdxRadiusSearch[i] ].x 
                    << " " << (*cloud)[ pointIdxRadiusSearch[i] ].y 
                    << " " << (*cloud)[ pointIdxRadiusSearch[i] ].z 
                    << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;

            // 设为黄色
            (*cloud)[ pointIdxRadiusSearch[i] ].r = 255;
            (*cloud)[ pointIdxRadiusSearch[i] ].g = 255;
            (*cloud)[ pointIdxRadiusSearch[i] ].b = 0;
        }
    }

    // 将搜索点添加到点云中
    cloud->push_back(voxelSearchPoint);
    cloud->push_back(nearestKSearchPoint);
    cloud->push_back(radiusSearchPoint);

    // 可视化点云
    pcl::visualization::CloudViewer viewer("PointCloud Viewer");
    viewer.showCloud(cloud);

    // 主循环，直到窗口关闭
    while (!viewer.wasStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}