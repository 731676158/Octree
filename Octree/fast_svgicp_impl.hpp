#ifndef FAST_GICP_FAST_SVGICP_IMPL_HPP
#define FAST_GICP_FAST_SVGICP_IMPL_HPP

#include <omp.h>

#include <atomic>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>

#include "so3/so3.hpp"
#include "fast_vgicp.hpp"

namespace fast_gicp {

template <typename PointSource, typename PointTarget>
FastSVGICP<PointSource, PointTarget>::FastSVGICP() : FastGICP<PointSource, PointTarget>() {
  this->reg_name_ = "FastVGICP";

  source_voxel_resolution_ = 1.0;
  target_voxel_resolution_ = 1.0;
  search_method_ = NeighborSearchMethod::DIRECT1;
  voxel_mode_ = VoxelAccumulationMode::ADDITIVE;
}

template <typename PointSource, typename PointTarget>
FastSVGICP<PointSource, PointTarget>::~FastSVGICP() {}

template <typename PointSource, typename PointTarget>
void FastSVGICP<PointSource, PointTarget>::setSourceResolution(double resolution) {
  source_voxel_resolution_ = resolution;
}

template <typename PointSource, typename PointTarget>
void FastSVGICP<PointSource, PointTarget>::setTargetResolution(double resolution) {
  target_voxel_resolution_ = resolution;
}

template <typename PointSource, typename PointTarget>
void FastSVGICP<PointSource, PointTarget>::setNeighborSearchMethod(NeighborSearchMethod method) {
  search_method_ = method;
}

template <typename PointSource, typename PointTarget>
void FastSVGICP<PointSource, PointTarget>::setVoxelAccumulationMode(VoxelAccumulationMode mode) {
  voxel_mode_ = mode;
}

template <typename PointSource, typename PointTarget>
void FastSVGICP<PointSource, PointTarget>::swapSourceAndTarget() {
  input_.swap(target_);
  source_kdtree_.swap(target_kdtree_);
  source_covs_.swap(target_covs_);
  source_voxelmap_.swap(target_voxelmap_);
  voxel_correspondences_.clear();
  voxel_mahalanobis_.clear();
}

template <typename PointSource, typename PointTarget>
void FastSVGICP<PointSource, PointTarget>::setInputSource(const PointCloudTargetConstPtr& cloud) {
    if (input_ == cloud) {
        return;
    }

    FastGICP<PointSource, PointTarget>::setInputSource(cloud);
    source_voxelmap_.reset();
}

template <typename PointSource, typename PointTarget>
void FastSVGICP<PointSource, PointTarget>::setInputTarget(const PointCloudTargetConstPtr& cloud) {
  if (target_ == cloud) {
    return;
  }

  FastGICP<PointSource, PointTarget>::setInputTarget(cloud);
  target_voxelmap_.reset();
}

template <typename PointSource, typename PointTarget>
void FastSVGICP<PointSource, PointTarget>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  source_voxelmap_.reset();
  target_voxelmap_.reset();

  FastGICP<PointSource, PointTarget>::computeTransformation(output, guess);
}

template <typename PointSource, typename PointTarget>
void FastSVGICP<PointSource, PointTarget>::update_correspondences(const Eigen::Isometry3d& trans) {
  voxel_correspondences_.clear();
  auto offsets = neighbor_offsets(search_method_);

  std::vector<std::vector<std::pair<int, GaussianVoxel::Ptr>>> corrs(num_threads_);
  for (auto& c : corrs) {
    c.reserve((input_->size() * offsets.size()) / num_threads_);
  }
  if (source_mean_cov_.empty()) source_mean_cov_.reserve(input_->size());

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int i = 0; i < input_->size(); i++) {
    /*auto source_coord = source_voxelmap_->voxel_coord(input_->at(i).getVector4fMap().template cast<double>());
    auto source_voxel = source_voxelmap_->lookup_voxel(source_coord);
    if (source_voxel->being_searched == true) continue;
    else source_voxel->being_searched = true;*/
    std::pair<Eigen::Vector4d, Eigen::Matrix4d> mean_cov_ = std::make_pair(
          Eigen::Vector4d::Zero(), Eigen::Matrix4d::Zero());
    int count_actual = 0;
    for (const auto& offset : offsets) {
      auto source_voxel= source_voxelmap_->lookup_voxel(source_voxelmap_->voxel_coord(input_->at(i).getVector4fMap().template cast<double>()) + offset);
      //source_voxel = source_voxelmap_->lookup_voxel(source_coord + offset);
      if (source_voxel != nullptr) {
        mean_cov_.first += source_voxel->mean;
        mean_cov_.second += source_voxel->cov;
        ++count_actual;
      }
    }
    mean_cov_.first /= count_actual;
    mean_cov_.second /= count_actual;
    source_mean_cov_.push_back(mean_cov_);

    Eigen::Vector4d transed_mean_A = trans * mean_cov_.first;
    Eigen::Vector3i target_coord = target_voxelmap_->voxel_coord(transed_mean_A);

    for (const auto& offset : offsets) {
      auto target_voxel = target_voxelmap_->lookup_voxel(target_coord + offset);
      if (target_voxel != nullptr) {
        corrs[omp_get_thread_num()].push_back(std::make_pair(i,target_voxel));
      }
    }
  }

  voxel_correspondences_.reserve(input_->size() * offsets.size());
  for (const auto& c : corrs) {
    voxel_correspondences_.insert(voxel_correspondences_.end(), c.begin(), c.end());
  }

  // precompute combined covariances
  voxel_mahalanobis_.resize(voxel_correspondences_.size());

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    const auto& cov_A = source_mean_cov_[corr.first].second;
    const auto& cov_B = corr.second->cov;

    Eigen::Matrix4d RCR = cov_B + trans.matrix() * cov_A * trans.matrix().transpose();
    RCR(3, 3) = 1.0;

    voxel_mahalanobis_[i] = RCR.inverse();
    voxel_mahalanobis_[i](3, 3) = 0.0;
  }
}

template <typename PointSource, typename PointTarget>
double FastSVGICP<PointSource, PointTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  if (source_voxelmap_ == nullptr) {
    source_voxelmap_.reset(new GaussianVoxelMap<PointSource>(source_voxel_resolution_, voxel_mode_));
    source_voxelmap_->create_voxelmap(*input_, source_covs_);
  }
  
  if (target_voxelmap_ == nullptr) {
    target_voxelmap_.reset(new GaussianVoxelMap<PointTarget>(target_voxel_resolution_, voxel_mode_));
    target_voxelmap_->create_voxelmap(*target_, target_covs_);
  }

  update_correspondences(trans);

  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = source_mean_cov_[corr.first].first;
    const auto& cov_A = source_mean_cov_[corr.first].second;

    const Eigen::Vector4d mean_B = corr.second->mean;
    const auto& cov_B = corr.second->cov;

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    double w = std::sqrt(target_voxel->num_points);
    sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error;

    if (H == nullptr || b == nullptr) {
      continue;
    }

    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp = dtdx0;

    Eigen::Matrix<double, 6, 6> Hi = w * jlossexp.transpose() * voxel_mahalanobis_[i] * jlossexp;
    Eigen::Matrix<double, 6, 1> bi = w * jlossexp.transpose() * voxel_mahalanobis_[i] * error;

    int thread_num = omp_get_thread_num();
    Hs[thread_num] += Hi;
    bs[thread_num] += bi;
  }

  if (H && b) {
    H->setZero();
    b->setZero();
    for (int i = 0; i < num_threads_; i++) {
      (*H) += Hs[i];
      (*b) += bs[i];
    }
  }

  return sum_errors;
}

template <typename PointSource, typename PointTarget>
double FastSVGICP<PointSource, PointTarget>::compute_error(const Eigen::Isometry3d& trans) {
  double sum_errors = 0.0;
#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = source_mean_cov_[corr.first].first;
    const auto& cov_A = source_mean_cov_[corr.first].second;

    const Eigen::Vector4d mean_B = corr.second->mean;
    const auto& cov_B = corr.second->cov;

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    double w = std::sqrt(target_voxel->num_points);
    sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error;
  }

  return sum_errors;
}

}  // namespace fast_gicp

#endif
