#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
// #include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "voxel_grid_feature.h"

using namespace std;
using namespace std::chrono;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace Eigen;

//#include "fast_vgicp.hpp"


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

void RegViewer(const PointCloud<PointXYZ>::ConstPtr& origin, const PointCloud<PointXYZ>::ConstPtr& featured)
{
	std::shared_ptr<PCLVisualizer> viewer(new PCLVisualizer("origin(red),featured(green)"));
	viewer->setBackgroundColor(255, 255, 255);
	PointCloudColorHandlerCustom<PointXYZ> origin_cloud_handler(origin, 255, 0, 0);
	PointCloudColorHandlerCustom<PointXYZ> featured_cloud_handler(featured, 0, 255, 0);
	viewer->addPointCloud<PointXYZ>(origin, origin_cloud_handler, "origin");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "origin");
	viewer->addPointCloud<PointXYZ>(featured, featured_cloud_handler, "featured");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "featured");
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

void voxel_sample(const PointCloud<PointXYZ>::ConstPtr& cloud, PointCloud<PointXYZ>::Ptr& filtered, float* res, bool remain, float sample_ratio)
{
	VoxelGridFeature<PointXYZ> voxelgrid;
	voxelgrid.setLeafSize(res[0], res[1], res[2]);
	voxelgrid.setInputCloud(cloud);
	voxelgrid.setRemainFeature(remain);
	voxelgrid.setSampleRatio(sample_ratio);
	voxelgrid.filter(*filtered);
}

//��׼
template<typename Registration>
pair<double, double> pcl_align(string category, Registration& reg, const PointCloud<PointXYZ>::ConstPtr& source,
	const PointCloud<PointXYZ>::ConstPtr& target, Matrix4f trans)
{
    cout <<"--------"<< category<<"---------"<<endl;
	PointCloud<PointXYZ>::Ptr aligned(new PointCloud<PointXYZ>());
	double score = 0.0;

	//�����trans��Ϊ��׼����
	//Matrix4f trans;
	//PointCloud<PointXYZ>::Ptr guessed(new PointCloud<PointXYZ>());
	//transformPointCloud(*source, *guessed, trans);

	size_t iter_times = 0;

	auto t1 = high_resolution_clock::now();
	reg.setInputSource(source);
	reg.setInputTarget(target);
	//reg.align(*aligned);
	//reg.align(*aligned, init_guess);
	reg.align(*aligned, trans);
	auto t2 = high_resolution_clock::now();
	double d = duration_cast<nanoseconds>(t2 - t1).count() / 1e6;

	cout << "time: " << d << " [msecs]." << endl;


	//NDT�õ�
	// iter_times = reg.getFinalNumIteration();
	// cout << "iter times: " << iter_times << endl;
	//��������
	//auto crit = reg.getConvergeCriteria();
	//cout << "ConvergeCriteria: " << crit << endl;
	//��׼�ɼ�
	score = reg.getFitnessScore();
	cout << "score: " << score << "[m^2]." << endl;
	trans = reg.getFinalTransformation();
	//cout << "transformation:" << endl << trans << endl;
	string aligned_name;

	//save
	//aligned_name = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\test_aligned" + rtime + ".pcd";
	//savePCDFile(aligned_name, *aligned);

	RegViewer(source, target, aligned);
	//RegViewer(guessed, target, aligned);
	return pair<double, double> {score, d};
}


int main()
{
	/*********************************���ƶ�ȡ���˲�*********************************************/

	string sourcefile, targetfile;
#if defined(_WIN32)
	//sourcefile = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\room_scan\\room_scan1.pcd";
	//targetfile = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\room_scan\\room_scan2.pcd";
	//sourcefile = "C:\\files\\codes\\git\\Octree\\source.pcd";
	//targetfile = "C:\\files\\codes\\git\\Octree\\target.pcd";
	//sourcefile = "E:\\�⼸������߰���\\һ��50��\\test1.pcd";
	//targetfile = "E:\\�⼸������߰���\\һ��50��\\test2_noise.pcd";
	sourcefile = "C:\\files\\codes\\git\\Octree\\data\\2011_09_26_drive_0005_sync\\pcds\\0000000010.pcd";
	targetfile = "C:\\files\\codes\\git\\Octree\\data\\2011_09_26_drive_0005_sync\\pcds\\0000000000.pcd";
#elif defined(__linux__)
	sourcefile = "/home/jeff/codes/Octree/data/2011_09_26_drive_0005_sync/pcds/0000000005.pcd";
	targetfile = "/home/jeff/codes/Octree/data/2011_09_26_drive_0005_sync/pcds/0000000000.pcd";
	//sourcefile = "/home/jeff/codes/Octree/room_scan2.pcd";
	//targetfile = "/home/jeff/codes/Octree/room_scan1.pcd";
#endif

    PointCloud<PointXYZ>::Ptr source(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr target(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr source_ori(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr target_ori(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr source_fea(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr target_fea(new PointCloud<PointXYZ>());

	cout << "Start reading" << endl;
	auto t1 = high_resolution_clock::now();
	if (loadPCDFile<PointXYZ>(sourcefile, *source) == -1)
	{
		PCL_ERROR("Couldn't read source file \n");
		return -1;
	}
	auto t2 = high_resolution_clock::now();
	if (loadPCDFile<PointXYZ>(targetfile, *target) == -1)
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
	CloudMessages(source);
	cout << "---target cloud messages: " << endl;
	CloudMessages(target);

	float res[3] = {0.1, 0.1, 0.1};
	voxel_sample(source, source_ori, res, false, 0);
	voxel_sample(target, target_ori, res, false, 0);
	float res1[3] = {1.0, 1.0, 1.0};
    voxel_sample(source, source_fea, res1, true, 0.5f);
	voxel_sample(target, target_fea, res1, true, 0.5f);

	cout << "-------------Original filtering" << endl;
	cout << "---source cloud messages: " << endl;
	CloudMessages(source_ori);
	cout << "---target cloud messages: " << endl;
	CloudMessages(target_ori);

    cout << "-------------Featured filtering" << endl;
	cout << "---source cloud messages: " << endl;
	CloudMessages(source_fea);
	cout << "---target cloud messages: " << endl;
	CloudMessages(target_fea);

	RegViewer(source_ori, source_fea);
	RegViewer(target_ori, target_fea);

	/***********************************�˲��˲���**************************************************/
	
	//NormalDistributionsTransform<PointXYZ, PointXYZ> pcl_ndt;
	//IterativeClosestPoint<PointXYZ, PointXYZ> pcl_icp;
	//fast_gicp::FastVGICP<PointXYZ, PointXYZ> vgicp;
	//fast_gicp::FastSVGICP<PointXYZ, PointXYZ> svgicp;
	GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> pcl_gicp;

	////ndt
	//pcl_ndt.setResolution(3.0);     //ע��ֱ���
	//pcl_ndt.setStepSize(0.1);
	//pcl_ndt.setMaximumIterations(70);
	//pcl_ndt.setTransformationEpsilon(0.001);// *target_tree_level_res[i]);     //������������趨��С����ֵ��

	//vgicp
	//vgicp.setResolution(0.8);     //ע��ֱ���
	//vgicp.setNumThreads(8);
	//vgicp.setTransformationEpsilon(0.001);// *target_tree_level_res[i]);     //������������趨��С����ֵ��
	//vgicp.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);

	////icp
	//pcl_icp.setTransformationEpsilon(0.001);// *target_tree_level_res[i]);     //������������趨��С����ֵ��

	//gicp
	pcl_gicp.setTransformationEpsilon(0.01);// *target_tree_level_res[i]);     //������������趨��С����ֵ��
	pcl_gicp.setMaximumIterations(35);
	/***********************************************************************************************/

	Matrix4f trans = Matrix4f::Identity();

	////����λ�˹���
	// AngleAxisf init_rotation(0.6931, Vector3f::UnitZ());
	// Translation3f init_translation(1.79387, 0.720047, 0);
	// Matrix4f init_guess = (init_translation * init_rotation).matrix();

	// PointCloud<PointXYZ>::Ptr guessed(new PointCloud<PointXYZ>());
	// pcl::transformPointCloud(*source_pre, *guessed, init_guess);

	// ��ͼ
    pcl_align("not sampled: ", pcl_gicp, source, target, trans);
    pcl_align("original sampled", pcl_gicp, source_ori, target_ori, trans);
    pcl_align("feature sampled", pcl_gicp, source_fea, target_fea, trans);
	//res_scts.push_back(pcl_align(pcl_ndt, source_pre, target_pre, trans));
	//res_scts.push_back(pcl_align(vgicp, source_pre, target_pre, trans));
	//res_scts.push_back(pcl_align(svgicp, source_pre, target_pre, trans));
	//res_scts.push_back(pcl_align(pcl_icp, guessed, target_pre, trans));
	//res_scts.push_back(pcl_align(pcl_gicp, source_pre, target_pre, trans));
	//drawRes(res_scts);

	//Matrix4f trans = Matrix4f::Identity();
	//pcl_align(pcl_ndt, source_pre, target_pre, trans);

	//PointCloud<PointXYZ>::Ptr aligned(new PointCloud<PointXYZ>());
	//pcl::transformPointCloud(*source_pre, *aligned, trans);

	//RegViewer(source_pre, target_pre, aligned);


}