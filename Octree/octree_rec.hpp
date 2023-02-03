#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>
#include <algorithm>
#include <Eigen/Dense>
#include <omp.h>
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

#define CloudData PointXYZ

namespace octReg {
	template <typename CloudType>
	class OctreeReg {
	public:
		void obtainCloud(PointCloud<CloudType>::Ptr source, PointCloud<CloudType>::Ptr target);
		void setRes(double res) { res_ = res; }
		void setLevel(int level) { level_ = level; }
		int getLevel() { return level_; }

	private:

		int level_;
		double res_;

		// Registration method
		//NormalDistributionsTransform<CloudData, CloudData> pcl_ndt;
		//IterativeClosestPoint<CloudData, CloudData> pcl_icp;
		GeneralizedIterativeClosestPoint<CloudType, CloudType> reg_;
		//fast_gicp::FastVGICP<CloudData, CloudData> vgicp;
		//fast_gicp::FastSVGICP<CloudData, CloudData> svgicp;

	};
} // namespace octReg
#pragma once
