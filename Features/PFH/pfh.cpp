// https://pcl.readthedocs.io/projects/tutorials/en/master/pfh_estimation.html#pfh-estimation
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_plotter.h>

int main()
{
    //------------------加载点云数据-------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../pcd_files/rabbit_downsampled.pcd", *cloud) == -1)
	{
		PCL_ERROR("Could not read file\n");
	}

	//------------------计算法线----------------------
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//ne.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
	// ne.setRadiusSearch(0.3);//半径搜素
	ne.compute(*normals);//开始进行法向计

    // Create the PFH estimation class, and pass the input dataset+normals to it
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (cloud);
    pfh.setInputNormals (normals);
    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
    pfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfh.setRadiusSearch (0.5);

    // Compute the features
    pfh.compute (*pfhs);

    // pfhs->size () should have the same size as the input cloud->size ()*

    // 直方图可视化
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*pfhs, 125, "cloud", 640, 200);
    plotter.plot();

    return 0;
}