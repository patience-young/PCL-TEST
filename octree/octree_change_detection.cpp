#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <chrono>
#include <thread>
#include <set>

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB>);

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

    float resolution = 32.0f;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree (resolution);

    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();

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
            std::cout << i << "# Index:" << pointIdxRadiusSearch[i]
                    << "    "  <<   (*cloud)[ pointIdxRadiusSearch[i] ].x 
                    << " " << (*cloud)[ pointIdxRadiusSearch[i] ].y 
                    << " " << (*cloud)[ pointIdxRadiusSearch[i] ].z << std::endl;

            // 设为黄色
            (*cloud)[ pointIdxRadiusSearch[i] ].r = 255;
            (*cloud)[ pointIdxRadiusSearch[i] ].g = 255;
            (*cloud)[ pointIdxRadiusSearch[i] ].b = 0;
        }
    }

    // 将除了搜索到的点都添加到cloudB
    std::set<int> pointIdxSet(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
    for (int i = 0; i < cloud->size(); i++) {
        if (pointIdxSet.find(i) != pointIdxSet.end()) continue;

        cloudB->push_back((*cloud)[i]);
    }

    // 将搜索点添加到点云中
    cloud->push_back(radiusSearchPoint);

    // 可视化点云
    pcl::visualization::CloudViewer viewer("PointCloud Viewer");
    viewer.showCloud(cloud);

    // 主循环，直到窗口关闭
    while (!viewer.wasStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Instantiate octree-based point cloud change detection class
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree_change (resolution);
    
    // Add points from cloudB to octree
    octree_change.setInputCloud (cloudB);
    octree_change.addPointsFromInputCloud ();

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree_change.switchBuffers ();

    // Add points from cloud to octree
    octree_change.setInputCloud (cloud);
    octree_change.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;

    // 获取cloudB中没有但cloud中有的点（即前面搜索到的点）
    octree_change.getPointIndicesFromNewVoxels (newPointIdxVector);

    // Output cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Output points
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    for (std::size_t i = 0; i < newPointIdxVector.size (); ++i) {
        std::cout << i << "# Index:" << newPointIdxVector[i]
                << "  Point:" << (*cloud)[newPointIdxVector[i]].x << " "
                << (*cloud)[newPointIdxVector[i]].y << " "
                << (*cloud)[newPointIdxVector[i]].z << std::endl;

        cloud_output->push_back((*cloud)[newPointIdxVector[i]]);
    }

    cloud_output->push_back(radiusSearchPoint);

    // 可视化点云
    pcl::visualization::CloudViewer viewerB("PointCloudB Viewer");
    viewerB.showCloud(cloud_output);

    // 主循环，直到窗口关闭
    while (!viewerB.wasStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}