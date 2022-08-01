#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>
#include <algorithm>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "fast_vgicp.hpp"

#include "octree_recur_iter.h"
#include "plot.h"


using namespace std;
using namespace std::chrono;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace pcl::octree;
using namespace Eigen;

int reg_times = 0;

void RegViewer(const PointCloud<PointXYZ>::ConstPtr& source, const PointCloud<PointXYZ>::ConstPtr& target)
{
	std::shared_ptr<PCLVisualizer> viewer(new PCLVisualizer("source(red),target(green)"));
	viewer->setBackgroundColor(255, 255, 255);
	PointCloudColorHandlerCustom<PointXYZ> source_cloud_handler(source, 255, 0, 0);
	PointCloudColorHandlerCustom<PointXYZ> target_cloud_handler(target, 0, 255, 0);
	viewer->addPointCloud<PointXYZ>(source, source_cloud_handler, "source");
	viewer->addPointCloud<PointXYZ>(target, target_cloud_handler, "target");
	viewer->spin();
}

void RegViewer(const PointCloud<PointXYZ>::ConstPtr& source, const PointCloud<PointXYZ>::ConstPtr& target, const PointCloud<PointXYZ>::ConstPtr& aligned)
{
	std::shared_ptr<PCLVisualizer> viewer(new PCLVisualizer("source(red),target(green),aligned(blue)"));
	viewer->setBackgroundColor(255, 255, 255);
	PointCloudColorHandlerCustom<PointXYZ> source_cloud_handler(source, 255, 0, 0);
	PointCloudColorHandlerCustom<PointXYZ> target_cloud_handler(target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZ> aligned_cloud_handler(aligned, 0, 0, 255);
	viewer->addPointCloud<PointXYZ>(source, source_cloud_handler, "source");
	viewer->addPointCloud<PointXYZ>(target, target_cloud_handler, "target");
	viewer->addPointCloud<PointXYZ>(aligned, aligned_cloud_handler, "aligned");
	viewer->spin();
}


//体素降采样
void voxel_sample(const PointCloud<PointXYZ>::ConstPtr& cloud, PointCloud<PointXYZ>::Ptr& filted, float* res)
{
	//PointCloud<PointXYZ>::Ptr filted(new PointCloud<PointXYZ>);

	ApproximateVoxelGrid<PointXYZ> voxelgrid;
	voxelgrid.setLeafSize(res[0], res[1], res[2]);
	voxelgrid.setInputCloud(cloud);
	voxelgrid.filter(*filted);
}

//ndt配准
template<typename Registration>
double pcl_align(Registration& reg, const PointCloud<PointXYZ>::ConstPtr& source,
	const PointCloud<PointXYZ>::ConstPtr& target, Matrix4f& trans)
{
	cout << "---------" << reg_times << "----------" << endl;
	PointCloud<PointXYZ>::Ptr aligned(new PointCloud<PointXYZ>());
	double score = 0.0;
	//Matrix4f trans;

	//AngleAxisf init_rotation(0.6931, Vector3f::UnitZ());
	//Translation3f init_translation(1.79387, 0.720047, 0);
	//Matrix4f init_guess = (init_translation * init_rotation).matrix();

	//PointCloud<PointXYZ>::Ptr guessed(new PointCloud<PointXYZ>());
	//transformPointCloud(*source, *guessed, trans);

	double fitness_score = 0.0;
	size_t iter_times = 0;

	auto t1 = high_resolution_clock::now();
	reg.setInputSource(source);
	reg.setInputTarget(target);
	reg.align(*aligned);
	//reg.align(*aligned, init_guess);
	auto t2 = high_resolution_clock::now();
	double d = duration_cast<nanoseconds>(t2 - t1).count() / 1e9;
	cout << "time: " << d << " [secs]." << endl;


	//NDT用到
	//iter_times = reg.getFinalNumIteration();
	//cout << "iter times: " << iter_times << endl;
	//收敛条件
	//auto crit = reg.getConvergeCriteria();
	//cout << "ConvergeCriteria: " << crit << endl;
	//配准成绩
	score = reg.getFitnessScore();
	cout << "score: " << score << "[m^2]." << endl;
	trans = reg.getFinalTransformation();
	cout << "transformation:" << endl << trans << endl;
	string aligned_name, rtime;
	rtime = to_string(reg_times);
	aligned_name = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\test_aligned" + rtime + ".pcd";

	savePCDFile(aligned_name, *aligned);

	RegViewer(source, target, aligned);
	//RegViewer(guessed, target, aligned);

	++reg_times;
	return d;
}

int main()
{

	/*********************************点云读取与滤波*********************************************/

	string sourcefile, targetfile;
	//sourcefile = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\room_scan\\room_scan1.pcd";
	//targetfile = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\room_scan\\room_scan2.pcd";
	sourcefile = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\source.pcd";
	targetfile = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\target.pcd";
	//sourcefile = "D:\\这几天的乱七八糟\\一组50个\\test1.pcd";
	//targetfile = "D:\\这几天的乱七八糟\\一组50个\\test2_noise.pcd";

	PointCloud<PointXYZ>::Ptr source_pre(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr target_pre(new PointCloud<PointXYZ>());

	cout << "Start reading" << endl;
	auto t1 = high_resolution_clock::now();
	if (loadPCDFile<PointXYZ>(sourcefile, *source_pre) == -1)
	{
		PCL_ERROR("Couldn't read source file \n");
		return -1;
	}
	auto t2 = high_resolution_clock::now();
	if (loadPCDFile<PointXYZ>(targetfile, *target_pre) == -1)
	{
		PCL_ERROR("Couldn't read target file \n");
		return -1;
	}
	auto t3 = high_resolution_clock::now();
	cout << "End reading" << endl;

	double ds = duration_cast<nanoseconds>(t2 - t1).count() / 1e9;
	double dt = duration_cast<nanoseconds>(t3 - t2).count() / 1e9;

	cout << "Loaded " << source_pre->width * source_pre->height << " source points and " << target_pre->width * target_pre->height << " target points." << endl;
	cout << "Source time: " << ds << " sec. Target time: " << dt << " sec." << endl;

	/*********************************体素滤波*********************************************/

	//降采样
	//float res[3] = { 0.05, 0.05, 0.05 };
	PointCloud<PointXYZ>::Ptr source(new PointCloud<PointXYZ>());
	//voxel_sample(source_pre, source, res);
	PointCloud<PointXYZ>::Ptr target(new PointCloud<PointXYZ>());
	//voxel_sample(target_pre, target, res);
	//cout << "Left " << source->width * source->height << " source points and " << target->width * target->height << " target points." << endl;

	//RegViewer(source_pre, source);

	///*********************************建立八叉树*********************************************/
	//OctreePointCloudVoxelCentroid<PointXYZ> octree_voxels(1.0);
	//octree_voxels.setInputCloud(source);
	//octree_voxels.addPointsFromInputCloud();
	//int source_depth = octree_voxels.getTreeDepth();
	//cout << "source depth : " << source_depth << endl;
	//vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>> occupied_voxel_centroids;
	//int occupied_voxnum = octree_voxels.getOccupiedVoxelCenters(occupied_voxel_centroids); 

	//cout << "occupied voxels: " << occupied_voxnum << endl;

	/*********************************按分辨率循环配准*********************************************/

	PointCloud<PointXYZ>::Ptr last_source(new PointCloud<PointXYZ>(*source));

	PointCloud<PointXYZ>::Ptr source_temp(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr target_temp(new PointCloud<PointXYZ>());

	cout << "--- pcl_ndt ---" << endl;
	//NormalDistributionsTransform<PointXYZ, PointXYZ> pcl_ndt;
	//IterativeClosestPoint<PointXYZ, PointXYZ> pcl_icp;
	fast_gicp::FastVGICP<PointXYZ, PointXYZ> vgicp;

	Matrix4f trans = Matrix4f::Identity();
	Matrix4f trans_iter = Matrix4f::Identity();

	//double interval = (5 - 0.001) / 10.0;

	//for (double res = 5; res >= 0.05; res /= 2.0)// res -= interval)
	//{
	//	//源点云
	//	OctreePointCloud<PointXYZ> octree_source(res);
	//	octree_source.setInputCloud(last_source);
	//	octree_source.addPointsFromInputCloud();
	//	vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>> occupied_source_centroids;
	//	octree_source.getOccupiedVoxelCenters(occupied_source_centroids);
	//	source_temp->height = 1;
	//	source_temp->width = occupied_source_centroids.size();
	//	source_temp->is_dense = false;
	//	source_temp->points = occupied_source_centroids;
	//	
	//	//目标点云
	//	OctreePointCloud<PointXYZ> octree_target(res);
	//	octree_target.setInputCloud(target);
	//	octree_target.addPointsFromInputCloud();
	//	vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>> occupied_target_centroids;
	//	octree_target.getOccupiedVoxelCenters(occupied_target_centroids);
	//	target_temp->height = 1;
	//	target_temp->width = occupied_target_centroids.size();
	//	target_temp->is_dense = false;
	//	target_temp->points = occupied_target_centroids;
	//
	//	//配准
	//	pcl_ndt.setResolution(5 * res);
	//	pcl_ndt.setTransformationEpsilon(0.005 * res);
	//	Matrix4f trans_iter = Matrix4f::Identity();
	//	pcl_align(pcl_ndt, source_temp, target_temp, trans_iter);
	//	transformPointCloud(*last_source, *last_source, trans_iter);
	//
	//	trans *= trans_iter;
	//}

		/*********************************八叉树节点构建*********************************************/

	//double res_octree = 0.05;
	//OctreePointCloud<PointXYZ> octree_source(res_octree);
	//octree_source.setInputCloud(last_source);
	//octree_source.addPointsFromInputCloud();

	//OctreePointCloud<PointXYZ> octree_target(res_octree);
	//octree_target.setInputCloud(target);
	//octree_target.addPointsFromInputCloud();

	//vector<OctreePointCloud<PointXYZ>::AlignedPointTVector> occupied_centers_source_treelevel;
	//vector<OctreePointCloud<PointXYZ>::AlignedPointTVector> occupied_centers_target_treelevel;
	//vector<float> source_tree_level_res;
	//vector<float> target_tree_level_res;

	//size_t stl = octree_source.getTreeDepth();
	//size_t ttl = octree_target.getTreeDepth();

	//size_t depth = min(stl, ttl);
	//source_tree_level_res.resize(depth);
	//target_tree_level_res.resize(depth);


	//单线程
	//auto s1 = high_resolution_clock::now();
	//GetOctreeLevelCentroidsVector<OctreePointCloud<PointXYZ>> level_center_source_vec(octree_source, occupied_centers_source_treelevel, source_tree_level_res);
	//auto s2 = high_resolution_clock::now();
	//GetOctreeLevelCentroidsVector<OctreePointCloud<PointXYZ>> level_center_target_vec(octree_target, occupied_centers_target_treelevel, target_tree_level_res);
	//auto s3 = high_resolution_clock::now();

	////双线程
	//auto s1 = high_resolution_clock::now();
	//OctreeLevelContainer<OctreePointCloud<PointXYZ>> source_container(&octree_source, occupied_centers_source_treelevel, source_tree_level_res);
	//OctreeLevelContainer<OctreePointCloud<PointXYZ>> target_container(&octree_target, occupied_centers_target_treelevel, target_tree_level_res);
	//GetOctreeLevelCentroidsVector<OctreePointCloud<PointXYZ>> level_center_source_vec;
	//GetOctreeLevelCentroidsVector<OctreePointCloud<PointXYZ>> level_center_target_vec;
	//thread th1(level_center_source_vec, ref(source_container));
	//thread th2(level_center_target_vec, ref(target_container));
	//th1.join();
	//th2.join();
	//auto s2 = high_resolution_clock::now();

	//double source_read_time= duration_cast<nanoseconds>(s2 - s1).count() / 1e9;
	//double target_read_time= duration_cast<nanoseconds>(s3 - s2).count() / 1e9;
	//cout << "source_read_time: " << source_read_time << " [sec] target_read_time: " << target_read_time << " [sec]" << endl;

	//double read_time = duration_cast<nanoseconds>(s2 - s1).count() / 1e9;
	//cout << "read_time: " << read_time << " [sec]" << endl;
	//cout << "begin recursively registration" << endl;

	//vector<double> reg_times;
	//double reg_this_time = 0;

	//AngleAxisf init_rotation(0.6931, Vector3f::UnitZ());
	//Translation3f init_translation(1.79387, 0.720047, 0);
	//Matrix4f init_guess = (init_translation * init_rotation).matrix();

	//trans = init_guess;

	//pcl::transformPointCloud(*last_source, *source_temp, trans);
	//RegViewer(source, target, source_temp);

	//string source_name = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\source.pcd";
	//source_temp->height = 1;
	//source_temp->width = occupied_centers_source_treelevel[5].size();
	//source_temp->is_dense = false;
	//source_temp->points = occupied_centers_source_treelevel[5];
	//savePCDFile(source_name, *source_temp);
	//string target_name = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\target.pcd";
	//target_temp->height = 1;
	//target_temp->width = occupied_centers_target_treelevel[5].size();
	//target_temp->is_dense = false;
	//target_temp->points = occupied_centers_target_treelevel[5];
	//savePCDFile(target_name, *target_temp);

	//for (size_t i = 3; i < depth; i++)
	//{

	//	source_temp->height = 1;
	//	source_temp->width = occupied_centers_source_treelevel[i].size();
	//	source_temp->is_dense = false;
	//	source_temp->points = occupied_centers_source_treelevel[i];

	//	target_temp->height = 1;
	//	target_temp->width = occupied_centers_target_treelevel[i].size();
	//	target_temp->is_dense = false;
	//	target_temp->points = occupied_centers_target_treelevel[i];

	//	pcl::transformPointCloud(*source_temp, *source_temp, trans);

	//	//配准
	//	vgicp.setResolution(max(static_cast<float>(0.05), target_tree_level_res[i]));
	//	vgicp.setNumThreads(1);
	//	vgicp.setTransformationEpsilon(0.001);// *target_tree_level_res[i]);     //就先依照这个设定最小的阈值吧
	//	trans_iter = Matrix4f::Identity();

	//	//if (i == 6) trans_iter = init_guess;  //第一次进来添加一个初始位姿估计

	//	reg_this_time = pcl_align(vgicp, source_temp, target_temp, trans_iter);
	//	reg_times.push_back(reg_this_time);

	//	trans *= trans_iter;
	//}


	/*PointCloud<PointXYZ>::Ptr aligned(new PointCloud<PointXYZ>());
	pcl::transformPointCloud(*source, *aligned, trans);*/

	//RegViewer(source, target, aligned);

	/*********************************可视化*********************************************/
	//vector<double> reg_times;
	//reg_times.push_back(0.01);
	//reg_times.push_back(0.1);
	//reg_times.push_back(3);
	//reg_times.push_back(5);
	//reg_times.push_back(3);
	//reg_times.push_back(3);
	//reg_times.push_back(2);
	//reg_times.push_back(0.5);


	//int len = reg_times.size();
	//pythonInitial();
	//plot<double, int>(reg_times);
	//Py_Finalize();
	//system("pause");
	/*********************************调试*********************************************/
	//cout << "--- pcl_ndt ---" << endl;
	//NormalDistributionsTransform<PointXYZ, PointXYZ> pcl_ndt;
	vgicp.setResolution(6);  //10可以
	vgicp.setNumThreads(1);
	vgicp.setTransformationEpsilon(0.001);
	//pcl_ndt.setMaximumIterations(35);
	pcl_align(vgicp, source_pre, target_pre,trans);




}