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
#include "voxel_grid.h"
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace std::chrono;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace Eigen;

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


//���ؽ�����
void voxel_sample(const PointCloud<PointXYZ>::ConstPtr& cloud, PointCloud<PointXYZ>::Ptr& filtered, float* res, bool remain)
{
	VoxelGrid<PointXYZ> voxelgrid;
	voxelgrid.setLeafSize(res[0], res[1], res[2]);
	voxelgrid.setInputCloud(cloud);
    voxelgrid.setRemainFeature(remain);
	voxelgrid.filter(*filtered);
}

int main()
{
	/*********************************���ƶ�ȡ���˲�*********************************************/

	string cloudfile;
#if defined(_WIN32)
	cloudfile = "C:\\files\\codes\\git\\Octree\\data\\2011_09_26_drive_0005_sync\\pcds\\0000000010.pcd";
#elif defined(__linux__)
	cloudfile = "/home/jeff/codes/Octree/data/2011_09_26_drive_0005_sync/pcds/0000000005.pcd";
#endif

	PointCloud<PointXYZ>::Ptr source_pre(new PointCloud<PointXYZ>());

	cout << "Start reading" << endl;
	auto t1 = high_resolution_clock::now();
	if (loadPCDFile<PointXYZ>(cloudfile, *source_pre) == -1)
	{
		PCL_ERROR("Couldn't read source file \n");
		return -1;
	}
	auto t2 = high_resolution_clock::now();
	cout << "End reading" << endl;

	double ds = duration_cast<nanoseconds>(t2 - t1).count() / 1e9;

	cout << "Source load time: " << ds << " sec." << endl;
	cout << "---source cloud messages: " << endl;
	CloudMessages(source_pre);

    float res[3] = {1.0, 1.0, 1.0};
    PointCloud<PointXYZ>::Ptr filtered_remained(new PointCloud<PointXYZ>());
	voxel_sample(source_pre, filtered_remained, res, true);

    PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
	voxel_sample(source_pre, filtered, res, false);

	cout << "-------------After filtering" << endl;
	cout << "---features remained: " << endl;
	CloudMessages(filtered_remained);
    cout << "---plainful voxelized: " << endl;
    CloudMessages(filtered);

}