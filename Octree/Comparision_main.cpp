#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <pcl/common/common.h>
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


#if defined(_WIN32)
#include "plot.h"
#include "graph2d.h"
#endif


using namespace std;
using namespace std::chrono;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace pcl::octree;
using namespace Eigen;

/**
	���ļ������Բ�ͬ��׼�������Ա�

**/


int reg_times = 0;

//չʾ���ƻ�����Ϣ
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


//���ؽ�����
void voxel_sample(const PointCloud<PointXYZ>::ConstPtr& cloud, PointCloud<PointXYZ>::Ptr& filted, float* res)
{
	//PointCloud<PointXYZ>::Ptr filted(new PointCloud<PointXYZ>);

	ApproximateVoxelGrid<PointXYZ> voxelgrid;
	voxelgrid.setLeafSize(res[0], res[1], res[2]);
	voxelgrid.setInputCloud(cloud);
	voxelgrid.filter(*filted);
}

//��׼
template<typename Registration>
pair<double, double> pcl_align(Registration& reg, const PointCloud<PointXYZ>::ConstPtr& source,
	const PointCloud<PointXYZ>::ConstPtr& target, Matrix4f trans)
{
	cout << "---------" << reg_times << "----------" << endl;
	PointCloud<PointXYZ>::Ptr aligned(new PointCloud<PointXYZ>());
	double score = 0.0;

	//�����trans��Ϊ��׼����
	//Matrix4f trans;
	//PointCloud<PointXYZ>::Ptr guessed(new PointCloud<PointXYZ>());
	//transformPointCloud(*source, *guessed, trans);

	//����λ�˹���
	/*AngleAxisf init_rotation(0.6931, Vector3f::UnitZ());
	Translation3f init_translation(1.79387, 0.720047, 0);
	Matrix4f init_guess = (init_translation * init_rotation).matrix();*/



	size_t iter_times = 0;

	auto t1 = high_resolution_clock::now();
	reg.setInputSource(source);
	reg.setInputTarget(target);
	//reg.align(*aligned);
	//reg.align(*aligned, init_guess);
	reg.align(*aligned, trans);
	auto t2 = high_resolution_clock::now();
	double d = duration_cast<nanoseconds>(t2 - t1).count() / 1e6;

	cout << "Aligned " << source->width * source->height << " source points and " << target->width * target->height << " target points at this level." << endl;

	cout << "time: " << d << " [msecs]." << endl;


	//NDT�õ�
	//iter_times = reg.getFinalNumIteration();
	//cout << "iter times: " << iter_times << endl;
	//��������
	//auto crit = reg.getConvergeCriteria();
	//cout << "ConvergeCriteria: " << crit << endl;
	//��׼�ɼ�
	score = reg.getFitnessScore();
	cout << "score: " << score << "[m^2]." << endl;
	trans = reg.getFinalTransformation();
	cout << "transformation:" << endl << trans << endl;
	string aligned_name, rtime;
	rtime = to_string(reg_times);

	//save
	//aligned_name = "C:\\files\\point_cloud\\codes\\prt\\lecturePrt\\TreesAndKnn\\Octree\\test_aligned" + rtime + ".pcd";
	//savePCDFile(aligned_name, *aligned);

	RegViewer(source, target, aligned);
	//RegViewer(guessed, target, aligned);

	++reg_times;
	return pair<double, double> {score, d};
}

#if defined(_WIN32)
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
	for (int i = 0; i < res.size(); ++i)
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

	Graph2d::graph2d g2d_s(700, 590, { 0, scr_min}, { static_cast<double>(res.size()), scr_max});
	g2d_s.xlabel("level");
	g2d_s.ylabel("score");
	g2d_s.title("Reg_score_results");
	g2d_s.plot(scrs, RED);
	g2d_s.waitKey();

	/*Graph2d::graph2d g2d_t(700, 590, { 0, tm_min }, { static_cast<double>(res.size()), tm_max });
	g2d_t.xlabel("level");
	g2d_t.ylabel("time");
	g2d_t.title("Reg_time_results");
	g2d_t.plot(tms, RED);
	g2d_t.waitKey();*/
}
#endif

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
	sourcefile = "/home/jeff/codes/Octree/data/2011_09_26_drive_0005_sync/pcds/0000000015.pcd";
	targetfile = "/home/jeff/codes/Octree/data/2011_09_26_drive_0005_sync/pcds/0000000000.pcd";
#endif

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

	float res[3] = {0.1, 0.1, 0.1};
	voxel_sample(source_pre, source_pre, res);
	//voxel_sample(target_pre, target_pre, res);

	cout << "-------------After filtering" << endl;
	cout << "---source cloud messages: " << endl;
	CloudMessages(source_pre);
	cout << "---target cloud messages: " << endl;
	CloudMessages(target_pre);

	/***********************************�˲��˲���**************************************************/
	
	//NormalDistributionsTransform<PointXYZ, PointXYZ> pcl_ndt;
	//IterativeClosestPoint<PointXYZ, PointXYZ> pcl_icp;
	//fast_gicp::FastVGICP<PointXYZ, PointXYZ> vgicp;
	//fast_gicp::FastSVGICP<PointXYZ, PointXYZ> svgicp;
	GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> pcl_gicp;

	////ndt
	// pcl_ndt.setResolution(0.1);     //ע��ֱ���
	// pcl_ndt.setMaximumIterations(35);
	// pcl_ndt.setTransformationEpsilon(0.001);// *target_tree_level_res[i]);     //������������趨��С����ֵ��

	//vgicp
	//vgicp.setResolution(1.0);     //ע��ֱ���
	//vgicp.setNumThreads(8);
	//vgicp.setTransformationEpsilon(0.001);// *target_tree_level_res[i]);     //������������趨��С����ֵ��
	//vgicp.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);

	//svgicp
	//svgicp.setSourceResolution(0.01);
	//svgicp.setTargetResolution(0.05);     //ע��ֱ���
	//svgicp.setNumThreads(8);
	//svgicp.setTransformationEpsilon(0.001);// *target_tree_level_res[i]);     //������������趨��С����ֵ��
	//svgicp.setRotationEpsilon(0.001);
	//svgicp.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);

	////icp
	//pcl_icp.setTransformationEpsilon(0.001);// *target_tree_level_res[i]);     //������������趨��С����ֵ��

	////gicp
	pcl_gicp.setTransformationEpsilon(0.01);// *target_tree_level_res[i]);     //������������趨��С����ֵ��
	pcl_gicp.setMaximumIterations(35);
	/***********************************************************************************************/

	vector<pair<double, double>> res_scts;
	pair<double, double> sct;

	Matrix4f trans = Matrix4f::Identity();

	////����λ�˹���
	//AngleAxisf init_rotation(0.6931, Vector3f::UnitZ());
	//Translation3f init_translation(1.79387, 0.720047, 0);
	//Matrix4f init_guess = (init_translation * init_rotation).matrix();

	//PointCloud<PointXYZ>::Ptr guessed(new PointCloud<PointXYZ>());
	//pcl::transformPointCloud(*source_pre, *guessed, init_guess);

	// ��ͼ
	//res_scts.push_back(pcl_align(pcl_ndt, source_pre, target_pre, trans));
	//res_scts.push_back(pcl_align(vgicp, source_pre, target_pre, trans));
	//res_scts.push_back(pcl_align(svgicp, source_pre, target_pre, trans));
	//res_scts.push_back(pcl_align(pcl_icp, guessed, target_pre, trans));
	res_scts.push_back(pcl_align(pcl_gicp, source_pre, target_pre, trans));
	//drawRes(res_scts);

	//Matrix4f trans = Matrix4f::Identity();
	//pcl_align(pcl_ndt, source_pre, target_pre, trans);

	//PointCloud<PointXYZ>::Ptr aligned(new PointCloud<PointXYZ>());
	//pcl::transformPointCloud(*source_pre, *aligned, trans);

	//RegViewer(source_pre, target_pre, aligned);


}