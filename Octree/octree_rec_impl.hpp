#include "octree_rec.hpp"

namespace octReg {

	template <typename CloudType>
	void OctreeReg<CloudType>::createOctrees(PointCloud<CloudType>::Ptr source, PointCloud<CloudType>::Ptr target)
	{
		OctreePointCloud<CloudType> octree_source(res_);
		octree_source.setInputCloud(source);
		octree_source.addPointsFromInputCloud();
		OctreePointCloud<CloudType> octree_target(res_);
		octree_target.setInputCloud(target);
		octree_target.addPointsFromInputCloud();

		size_t stl = octree_source.getTreeDepth();
		size_t ttl = octree_target.getTreeDepth();
		depth_ = min(stl, ttl);

		// Ë«Ïß³Ì
		auto s1 = high_resolution_clock::now();
		OctreeLevelContainer<OctreePointCloud<CloudData>> source_container(&octree_source, occupied_centers_source_treelevel, depth_);
		OctreeLevelContainer<OctreePointCloud<CloudData>> target_container(&octree_target, occupied_centers_target_treelevel, depth_);
		GetOctreeLevelCentroidsVector<OctreePointCloud<CloudData>> level_center_source_vec;
		GetOctreeLevelCentroidsVector<OctreePointCloud<CloudData>> level_center_target_vec;
		thread th1(level_center_source_vec, ref(source_container));
		thread th2(level_center_target_vec, ref(target_container));
		th1.join();
		th2.join();
		auto s2 = high_resolution_clock::now();

	}

	template <typename CloudType>
	void OctreeReg<CloudType>::generatePoints(PointCloud<CloudType>::Ptr source, PointCloud<CloudType>::Ptr target)
	{
		source = new PointCloud<CloudType>();
		target = new PointCloud<CloudType>();

		source->height = 1;
		source->width = occupied_centers_source_treelevel[i].size();
		source->is_dense = false;
		source->points = occupied_centers_source_treelevel[i];

		target->height = 1;
		target->width = occupied_centers_target_treelevel[i].size();
		target->is_dense = false;
		target->points = occupied_centers_target_treelevel[i];
	}

	template <typename CloudType>
	double OctreeReg<CloudType>::registration(PointCloud<CloudType>::Ptr source, PointCloud<CloudType>::Ptr target, Matrix4f& trans)
	{
		reg_.setTransformationEpsilon(0.001);
		reg_.setMaximumOptimizerIterations(35);

		if (trans != Matrix4f::Identity())
		{
			trans = Matrix4f::Identity();
		}

		auto t1 = high_resolution_clock::now();
		reg_.setInputSource(source);
		reg_.setInputTarget(target);
		reg.aligned(*aligned);
		auto t2 = high_resolution_clock::now();
		double d = duration_cast<nanoseconds>(t2 - t1).count() / 1e9;

		double score = reg_.getFitnessScore();
		
		mess_.reg_single_time = d;
		mess_.reg_single_score = score;

		trans = reg_.getFinalTransformation();
	}
}


#pragma once
