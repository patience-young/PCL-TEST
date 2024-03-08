#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>	
using namespace std;

int main()
{
	//------------------加载点云数据-------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../pcd_files/rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Could not read file\n");
	}

	//------------下采样----------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_downsampled);

	//------------------计算法线----------------------
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//ne.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
	ne.setInputCloud(cloud_downsampled);
	ne.setSearchSurface(cloud);
	ne.setSearchMethod(tree);
	// ne.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
	ne.setRadiusSearch(0.3);//半径搜素
	ne.compute(*normals);//开始进行法向计
	
	//----------------可视化--------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normal viewer"));
	//viewer->initCameraParameters();//设置照相机参数，使用户从默认的角度和方向观察点云
	//设置背景颜色
	viewer->setBackgroundColor(0.3, 0.3, 0.3);
	//设置点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 225, 0);
	//添加坐标系
	viewer->addCoordinateSystem(0.1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_downsampled, single_color, "sample cloud");
	
 
    //添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，10表示需要显示法向的点云间隔，即每10个点显示一次法向，0.5表示法向长度。
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_downsampled, normals, 10, 0.5, "normals");
	//设置点云大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return 0;
}