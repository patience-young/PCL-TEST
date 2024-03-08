#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h> // for PCL_RANDOM_UNIFORM_01
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <cmath> // for M_PI
#include <random> // for std::uniform_real_distribution

int main() {
    // 生成一个球面点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB center(0.0, 0.0, 0.0, 255, 0, 0);
    float radius = 1.0;

    // 添加500个点到球面
    int num_points = 500;
    cloud->points.resize(num_points);
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0, 1.0);
    for (int i = 0; i < num_points; ++i) {
        float theta = distribution(generator) * 2.0f * M_PI;  // 随机生成角度
        float phi = distribution(generator) * M_PI;           // 随机生成角度
        float x = center.x + radius * sin(phi) * cos(theta);  // 根据球面参数计算点的坐标
        float y = center.y + radius * sin(phi) * sin(theta);
        float z = center.z + radius * cos(phi);
        cloud->points[i].x = x;
        cloud->points[i].y = y;
        cloud->points[i].z = z;
        cloud->points[i].r = 0;
        cloud->points[i].g = 0;
        cloud->points[i].b = 255;
    }

    // 添加噪声到点云
    float noise_stddev = 0.1;
    for (int i = 0; i < num_points; ++i) {
        std::normal_distribution<float> noise_distribution(0.0, noise_stddev);
        cloud->points[i].x += noise_distribution(generator);
        cloud->points[i].y += noise_distribution(generator);
        cloud->points[i].z += noise_distribution(generator);
    }

    // 创建球面模型
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = center.x;
    coefficients->values[1] = center.y;
    coefficients->values[2] = center.z;
    coefficients->values[3] = radius;

    // 创建投影对象
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_SPHERE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);

    // 创建投影后的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
    proj.filter(*cloud_projected);

    // Set color for projected points (green)
    for (auto& point: *cloud_projected)
    {
        point.r = 0;
        point.g = 255;
        point.b = 0;
    }

    // Visualize both clouds
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.addPointCloud(cloud, "original_cloud");
    // viewer.addPointCloud(cloud_projected, "projected_cloud");

    // Add coordinate axes
    viewer.addCoordinateSystem(1.0);

    while (!viewer.wasStopped())
        {
            viewer.spinOnce();
        }

    return (0);
}
