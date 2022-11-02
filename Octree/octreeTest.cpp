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
#include "fast_svgicp.hpp"

#include "octree_recur_iter.h"
#include "plot.h"

#include "graph2d.h"
#include <stdexcept>

using namespace std;
using namespace std::chrono;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace pcl::octree;
using namespace Eigen;

/**
	本文件用来对同一点云的不同层级做对比
	文件夹位置
	//C:\files\point_cloud\codes\prt\lecturePrt\TreesAndKnn\Octree\Octree

**/


int reg_times = 0;

//展示点云基本信息
void CloudMessages(const PointCloud<PointXYZ>::ConstPtr& cloud) {
	Eigen::Vector4f min_pt = Eigen::Vector4f::Zero();
	Eigen::Vector4f max_pt = Eigen::Vector4f::Zero();
	getMinMax3D<PointXYZ>(*cloud, min_pt, max_pt);
	cout << "Bound for the cloud: " << endl;
	cout << "x: [" << min_pt[0] << ", " << max_pt[0] << "];" << endl;
	cout << "y: [" << min_pt[1] << ", " << max_pt[1] << "];" << endl;
	cout << "z: [" << min_pt[2] << ", " << max_pt[2] << "];" << endl;
	cout << "Loaded " << cloud->width * cloud->height << " points." << endl;
}

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
pair<double,double> pcl_align(Registration& reg, const PointCloud<PointXYZ>::ConstPtr& source,
	const PointCloud<PointXYZ>::ConstPtr& target, Matrix4f& trans)
{
	cout << "---------" << reg_times << "----------" << endl;
	PointCloud<PointXYZ>::Ptr aligned(new PointCloud<PointXYZ>());
	double score = 0.0;

	//传入的trans作为配准估计
	//Matrix4f trans;
	//PointCloud<PointXYZ>::Ptr guessed(new PointCloud<PointXYZ>());
	//transformPointCloud(*source, *guessed, trans);
	
	//添加位姿估计
	/*AngleAxisf init_rotation(0.6931, Vector3f::UnitZ());
	Translation3f init_translation(1.79387, 0.720047, 0);
	Matrix4f init_guess = (init_translation * init_rotation).matrix();*/
	

	size_t iter_times = 0;

	auto t1 = high_resolution_clock::now();
	reg.setInputSource(source);
	reg.setInputTarget(target); 
	reg.align(*aligned);
	//reg.align(*aligned, init_guess);
	auto t2 = high_resolution_clock::now();
	double d = duration_cast<nanoseconds>(t2 - t1).count() / 1e9;

	cout << "Aligned " << source->width * source->height << " source points and " << target->width * target->height << " target points at this level." << endl;

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
	cout <<"transformation:" << endl << trans << endl;
	string aligned_name,rtime;
	rtime = to_string(reg_times);

	//save
	//aligned_name = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\test_aligned" + rtime + ".pcd";
	//savePCDFile(aligned_name, *aligned);

	RegViewer(source, target, aligned);
	//RegViewer(guessed, target, aligned);

	++reg_times;
	return pair<double, double> {score, d};
}

void drawRes(vector<pair<double, double>>& res)
{
	Graph2d::Point scr;
	Graph2d::Point tm;
	vector<Graph2d::Point> scrs;
	vector<Graph2d::Point> tms;
	double scr_min = DBL_MAX;
	double tm_min = DBL_MAX;
	double scr_max = DBL_MIN;
	double tm_max = DBL_MIN;
	for(int i = 0; i < res.size(); ++i)
	{
		scr.x = static_cast<double>(i);
		tm.x = static_cast<double>(i);
		scr.y = res[i].first;
		tm.y = res[i].second;
		scr_min = min(scr_min, scr.y);
		tm_min = min(tm_min, tm.y);
		scr_max = max(scr_max, scr.y);
		tm_max = max(tm_max, tm.y);
		scrs.push_back(scr);
		tms.push_back(tm);
	}

	/*Graph2d::graph2d g2d_s(700, 590, { 0, scr_min}, { static_cast<double>(res.size()), scr_max});
	g2d_s.xlabel("level");
	g2d_s.ylabel("score");
	g2d_s.title("Reg_score_results");
	g2d_s.plot(scrs, RED);
	g2d_s.waitKey();*/

	Graph2d::graph2d g2d_t(700, 590, { 0, tm_min }, { static_cast<double>(res.size()), tm_max });
	g2d_t.xlabel("level");
	g2d_t.ylabel("time");
	g2d_t.title("Reg_time_results");
	g2d_t.plot(tms, RED);
	g2d_t.waitKey();
}

int main()
{
	
	/*********************************点云读取与滤波*********************************************/
	
	string sourcefile, targetfile;
	//sourcefile = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\room_scan\\room_scan1.pcd";
	//targetfile = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\room_scan\\room_scan2.pcd";
	//sourcefile = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\source.pcd";
	//targetfile = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\target.pcd";
	//sourcefile = "D:\\这几天的乱七八糟\\一组50个\\test1.pcd";
	//targetfile = "D:\\这几天的乱七八糟\\一组50个\\test2_noise.pcd";
	sourcefile = "C:\\files\\codes\\git\\Octree\\data\\0000000001.pcd";
	targetfile = "C:\\files\\codes\\git\\Octree\\data\\0000000002.pcd";

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

	cout << "Source load time: " << ds << " sec.\n Target load time: " << dt << " sec." << endl;
	cout << "---source cloud messages: " << endl;
	CloudMessages(source_pre);
	cout << "---target cloud messages: " << endl;
	CloudMessages(target_pre);

	float res[3] = { 0.001, 0.001, 0.001 };
	voxel_sample(source_pre, source_pre, res);
	voxel_sample(target_pre, target_pre, res);

	cout << "-------------After filtering" << endl;
	cout << "---source cloud messages: " << endl;
	CloudMessages(source_pre);
	cout << "---target cloud messages: " << endl;
	CloudMessages(target_pre);
	
	/*********************************体素滤波*********************************************/
	
	//体素滤波降采样
	/*float res[3] = { 0.05, 0.05, 0.05 };
	PointCloud<PointXYZ>::Ptr source(new PointCloud<PointXYZ>());
	voxel_sample(source_pre, source, res);
	PointCloud<PointXYZ>::Ptr target(new PointCloud<PointXYZ>());
	voxel_sample(target_pre, target, res);
	cout << "Left " << source->width * source->height << " source points and " << target->width * target->height << " target points." << endl;*/

	//RegViewer(source_pre, source);
	
	//不采样了，直接用，因为后面有八叉树建立
	PointCloud<PointXYZ>::Ptr source(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr target(new PointCloud<PointXYZ>());
	source = source_pre;
	target = target_pre;
	cout << "Left " << source->width * source->height << " source points and " << target->width * target->height << " target points." << endl;

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
	NormalDistributionsTransform<PointXYZ, PointXYZ> pcl_ndt;
	//IterativeClosestPoint<PointXYZ, PointXYZ> pcl_icp;
	//fast_gicp::FastVGICP<PointXYZ, PointXYZ> vgicp;
	fast_gicp::FastSVGICP<PointXYZ, PointXYZ> svgicp;

	Matrix4f trans = Matrix4f::Identity();
	Matrix4f trans_iter = Matrix4f::Identity();
	/*********************************调试:直接配准*********************************************/
	//cout << "--- pcl_ndt ---" << endl;
	//NormalDistributionsTransform<PointXYZ, PointXYZ> pcl_ndt;
	//pcl_ndt.setResolution(6.0);
	//pcl_ndt.setStepSize(0.1);
	//pcl_ndt.setTransformationEpsilon(0.0001);
	//pcl_ndt.setMaximumIterations(35);
	//pcl_align(pcl_ndt, source, target,trans);
	
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

	double res_octree = 0.001;
	OctreePointCloud<PointXYZ> octree_source(res_octree);
	octree_source.setInputCloud(last_source);
	octree_source.addPointsFromInputCloud();

	OctreePointCloud<PointXYZ> octree_target(res_octree);
	octree_target.setInputCloud(target);
	octree_target.addPointsFromInputCloud();

	vector<OctreePointCloud<PointXYZ>::AlignedPointTVector> occupied_centers_source_treelevel;
	vector<OctreePointCloud<PointXYZ>::AlignedPointTVector> occupied_centers_target_treelevel;
	vector<float> source_tree_level_res;
	vector<float> target_tree_level_res;

	size_t stl = octree_source.getTreeDepth();
	size_t ttl = octree_target.getTreeDepth();

	size_t depth = min(stl,ttl);
	source_tree_level_res.resize(depth);
	target_tree_level_res.resize(depth);


	//单线程
	//auto s1 = high_resolution_clock::now();
	//GetOctreeLevelCentroidsVector<OctreePointCloud<PointXYZ>> level_center_source_vec(octree_source, occupied_centers_source_treelevel, source_tree_level_res);
	//auto s2 = high_resolution_clock::now();
	//GetOctreeLevelCentroidsVector<OctreePointCloud<PointXYZ>> level_center_target_vec(octree_target, occupied_centers_target_treelevel, target_tree_level_res);
	//auto s3 = high_resolution_clock::now();
	
	////双线程
	auto s1 = high_resolution_clock::now();
	OctreeLevelContainer<OctreePointCloud<PointXYZ>> source_container(&octree_source, occupied_centers_source_treelevel, source_tree_level_res);
	OctreeLevelContainer<OctreePointCloud<PointXYZ>> target_container(&octree_target, occupied_centers_target_treelevel, target_tree_level_res);
	GetOctreeLevelCentroidsVector<OctreePointCloud<PointXYZ>> level_center_source_vec;
	GetOctreeLevelCentroidsVector<OctreePointCloud<PointXYZ>> level_center_target_vec;
	thread th1(level_center_source_vec, ref(source_container));
	thread th2(level_center_target_vec, ref(target_container));
	th1.join();
	th2.join();
	auto s2 = high_resolution_clock::now();

	//double source_read_time= duration_cast<nanoseconds>(s2 - s1).count() / 1e9;
	//double target_read_time= duration_cast<nanoseconds>(s3 - s2).count() / 1e9;
	//cout << "source_read_time: " << source_read_time << " [sec] target_read_time: " << target_read_time << " [sec]" << endl;
	
	double read_time = duration_cast<nanoseconds>(s2 - s1).count() / 1e9;
	cout << "read_time: " << read_time << " [sec]" << endl;

	cout << "--------levels and resolution----------" << endl;
	for (int i = 1; i <= depth; ++i) {
		cout << "Level " << i << " resolution: " << target_tree_level_res[i - 1] << endl;
	}

	cout << "begin recursively registration" << endl;

	//vector<double> reg_times;
	//double reg_this_time = 0;

	//vector<double> reg_scores;
	//double reg_this_score = 0;

	vector<pair<double, double>> reg_score_times;
	pair<double, double> reg_this_st;

	//AngleAxisf init_rotation(0.6931, Vector3f::UnitZ());
	//Translation3f init_translation(1.79387, 0.720047, 0);
	//Matrix4f init_guess = (init_translation * init_rotation).matrix();

	//trans = init_guess;

	//pcl::transformPointCloud(*last_source, *source_temp, trans);
	//RegViewer(source, target, source_temp);

	for (size_t i =2; i < depth; i++)
	{
		
		source_temp->height = 1;
		source_temp->width = occupied_centers_source_treelevel[i].size();
		source_temp->is_dense = false;
		source_temp->points = occupied_centers_source_treelevel[i];

		target_temp->height = 1;
		target_temp->width = occupied_centers_target_treelevel[i].size();
		target_temp->is_dense = false;
		target_temp->points = occupied_centers_target_treelevel[i];

		cout << "Level " << i << " source points: " << occupied_centers_source_treelevel[i].size() << " target points: " << occupied_centers_target_treelevel[i].size() << endl;

		pcl::transformPointCloud(*source_temp, *source_temp, trans);

		//配准
		// ndt
		//pcl_ndt.setResolution(max(static_cast<float>(0.05) ,target_tree_level_res[i]));     //注意分辨率
		////pcl_ndt.setNumThreads(1);
		//pcl_ndt.setTransformationEpsilon(0.001);// *target_tree_level_res[i]);     //就先依照这个设定最小的阈值吧

		//svgicp
		svgicp.setSourceResolution(0.01);
		svgicp.setTargetResolution(0.05);     //注意分辨率
		svgicp.setNumThreads(8);
		svgicp.setTransformationEpsilon(0.001);// *target_tree_level_res[i]);     //就先依照这个设定最小的阈值吧
		svgicp.setRotationEpsilon(0.001);
		//svgicp.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);

		trans_iter = Matrix4f::Identity();
		
		//if (i == 6) trans_iter = init_guess;  //第一次进来添加一个初始位姿估计
		
		
		//reg_this_st = pcl_align(pcl_ndt, source_temp, target_temp, trans_iter);
		reg_this_st = pcl_align(svgicp, source_temp, target_temp, trans_iter);

		reg_score_times.push_back(reg_this_st);

		trans *= trans_iter;
	}


	PointCloud<PointXYZ>::Ptr aligned(new PointCloud<PointXYZ>());
	pcl::transformPointCloud(*source, *aligned, trans);

	drawRes(reg_score_times);

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



}