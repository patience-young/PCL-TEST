#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
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
    srand (time (NULL));

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
        (*cloud)[i].r = 255; // 默认颜色为白色
        (*cloud)[i].g = 255;
        (*cloud)[i].b = 255;
    }

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

    kdtree.setInputCloud (cloud);

    pcl::PointXYZRGB searchPoint1 = RandSearchPoint();

    // K nearest neighbor search

    int K = 10;

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);

    std::cout << "K nearest neighbor search at (" << searchPoint1.x 
            << " " << searchPoint1.y 
            << " " << searchPoint1.z
            << ") with K=" << K << std::endl;

    if ( kdtree.nearestKSearch (searchPoint1, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
    {
        for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i) {
            (*cloud)[pointIdxKNNSearch[i]].r = 0; // 将最近邻点着色为蓝色
            (*cloud)[pointIdxKNNSearch[i]].g = 0;
            (*cloud)[pointIdxKNNSearch[i]].b = 255;
        }
    }

    // Neighbors within radius search
    pcl::PointXYZRGB searchPoint2 = RandSearchPoint();
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 512.0f * rand () / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint2.x 
            << " " << searchPoint2.y 
            << " " << searchPoint2.z
            << ") with radius=" << radius << std::endl;


    if ( kdtree.radiusSearch (searchPoint2, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
            (*cloud)[pointIdxRadiusSearch[i]].r = 0; // 将半径内的邻居点着色为绿色
            (*cloud)[pointIdxRadiusSearch[i]].g = 255;
            (*cloud)[pointIdxRadiusSearch[i]].b = 0;
        }
    }

    // 将搜索点添加到点云中
    cloud->push_back(searchPoint1);
    cloud->push_back(searchPoint2);

    // 可视化点云
    pcl::visualization::CloudViewer viewer("PointCloud Viewer");
    viewer.showCloud(cloud);

    // 主循环，直到窗口关闭
    while (!viewer.wasStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  return 0;
}
