#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
using namespace std;

int main()
{
	//------------------加载点云数据-------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../pcd_files/rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Could not read file\n");
	}

	// Create a set of indices to be used. For simplicity, we're going to be using the first 10% of the points in cloud
	std::vector<int> indices (std::floor (cloud->size () / 10));
	for (std::size_t i = 0; i < indices.size (); ++i) {
		indices[i] = i;
	}

	//------------------计算法线----------------------
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//ne.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
	ne.setInputCloud(cloud);

	// Pass the indices
	pcl::IndicesPtr indicesptr (new std::vector<int> (indices));
	ne.setIndices (indicesptr);

	ne.setSearchMethod(tree);
	// ne.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
	ne.setRadiusSearch(0.3);//半径搜素
	ne.compute(*normals);//开始进行法向计

	//-------------为方便可视化，将前10%点云提出-------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_part(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, indices, *cloud_part);
	
	//----------------可视化--------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normal viewer"));
	//viewer->initCameraParameters();//设置照相机参数，使用户从默认的角度和方向观察点云
	//设置背景颜色
	viewer->setBackgroundColor(0.3, 0.3, 0.3);
	//设置点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 225, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_part(cloud_part, 0, 225, 0);
	//添加坐标系
	viewer->addCoordinateSystem(0.1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->addPointCloud<pcl::PointXYZ>(cloud_part, single_color_part, "part sample cloud");
 
    //添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，10表示需要显示法向的点云间隔，即每10个点显示一次法向，0.5表示法向长度。
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_part, normals, 10, 0.5, "normals");
	//设置点云大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "part sample cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return 0;
}