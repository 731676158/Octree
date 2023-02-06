#pragma once
#include <omp.h>
#include <vector>
#include <chrono>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/octree/octree_iterator.h>
using namespace std;
using namespace std::chrono;

namespace pcl {
	namespace octree {
		template <typename OctreeT>
		class OctreeLevelIterator : public OctreeIteratorBase<OctreeT> {
		public:
			// public typedefs
			using BranchNode = typename OctreeIteratorBase<OctreeT>::BranchNode;
			using LeafNode = typename OctreeIteratorBase<OctreeT>::LeafNode;
		
			/** \brief Empty constructor.
			 * \param[in] max_depth_arg Depth limitation during traversal
			 */
			explicit OctreeLevelIterator(unsigned int max_depth_arg = 0);
		
			/** \brief Constructor.
			 * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
			 * root node.
			 * \param[in] max_depth_arg Depth limitation during traversal
			 */
			explicit OctreeLevelIterator(OctreeT* octree_arg,
				unsigned int max_depth_arg = 0);
		
			/** \brief Constructor.
			 * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its
			 * root node.
			 * \param[in] max_depth_arg Depth limitation during traversal
			 * \param[in] current_state A pointer to the current iterator state
			 *
			 *  \warning For advanced users only.
			 */
			explicit OctreeLevelIterator(
				OctreeT* octree_arg,
				unsigned int max_depth_arg,
				IteratorState* current_state,
				const std::deque<IteratorState*>& fifo = std::deque<IteratorState*>())
				: OctreeIteratorBase<OctreeT>(octree_arg, max_depth_arg, current_state), FIFO_(fifo)
			{}
		
			/** \brief Copy Constructor.
			 * \param[in] other Another OctreeBreadthFirstIterator to copy from
			 */
			OctreeLevelIterator(const OctreeLevelIterator& other)
				: OctreeIteratorBase<OctreeT>(other), FIFO_(other.FIFO_)
			{
				this->current_state_ = FIFO_.size() ? FIFO_.front() : NULL;
			}
		
			/** \brief Copy operator.
			 * \param[in] src the iterator to copy into this
			 */
			inline OctreeLevelIterator&
				operator=(const OctreeLevelIterator& src)
			{
		
				OctreeIteratorBase<OctreeT>::operator=(src);
		
				FIFO_ = src.FIFO_;
		
				if (FIFO_.size()) {
					this->current_state_ = FIFO_.front();
				}
				else {
					this->current_state_ = 0;
				}
		
				return (*this);
			}
		
			/** \brief Reset the iterator to the root node of the octree
			 */
			void
				reset();
		
			/** \brief Preincrement operator.
			 * \note step to next octree node
			 */
			OctreeLevelIterator&
				operator++();
		
			/** \brief postincrement operator.
			 * \note step to next octree node
			 */
			inline OctreeLevelIterator
				operator++(int)
			{
				OctreeLevelIterator _Tmp = *this;
				++* this;
				return (_Tmp);
			}
		
		protected:
			/** FIFO list */
			std::deque<IteratorState*> FIFO_;
		};

		template<typename Octree = OctreePointCloud<PointXYZ>>
		struct OctreeLevelContainer
		{
			using AlignedPointTVector = std::vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>>;
			OctreeLevelContainer(Octree* oct_, std::vector<AlignedPointTVector>& vec_, std::vector<float>& res_vec_) :
				oct(oct_), vec(vec_), res_vec(res_vec_) {}

			Octree* oct;
			vector<AlignedPointTVector>& vec;
			vector<float>& res_vec;
		};

		template<typename Octree= OctreePointCloud<PointXYZ>>
		class GetOctreeLevelCentroidsVector :public OctreeBreadthFirstIterator<Octree>
		{
		public:
			using AlignedPointTVector = std::vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>>;
		public:
			GetOctreeLevelCentroidsVector() = default;
			GetOctreeLevelCentroidsVector(Octree& oct, std::vector<AlignedPointTVector>& vec, std::vector<float>& res_vec)
			{
				this->operator()(oct, vec, res_vec);
			}
			void operator()(Octree& oct, std::vector<AlignedPointTVector>& vec, std::vector<float>& res_vec)
			{
				//OctreeBreadthFirstIterator<Octree> it(&oct, 0);
				OctreeLevelIterator<Octree> it(&oct, 0);
				Eigen::Vector3f min_pt;
				Eigen::Vector3f max_pt;
				min_pt.setZero();
				max_pt.setZero();
				int current_level = 0;

				size_t dpt = res_vec.size();

				/*level_vec.resize(oct->getTreeDepth());
				res_level.resize(oct->getTreeDepth());*/
				level_vec.resize(dpt);
				res_level.resize(dpt);

				//auto ts1 = high_resolution_clock::now();
				//size_t i = 0;
				//while(it.current_state_)
				while ((it.current_state_) && (it.current_state_->depth_ <= dpt))
				{
					//i++;
					//auto tt1 = high_resolution_clock::now();
					pcl::PointXYZ p;// (new pcl::PointCloud<pcl::PointXYZ>());
					current_level = it.current_state_->depth_;

					if (current_level == 0)//根节点不存放格子了就
					{
						//auto root_it1 = high_resolution_clock::now();
						++it;
						//auto root_it2 = high_resolution_clock::now();
						//double root_d = duration_cast<nanoseconds>(root_it2 - root_it1).count()/1e6;
						//cout << "root_it++ time: " << root_d << " [msec]." << endl;
						continue;
					}

					oct.genVoxelCenterFromOctreeKey(it.current_state_->key_, current_level, p);

					
					//if (level_vec.size() != current_level)
					if (level_vec[current_level - 1].empty())
					{
						//依照本层节点的包围盒来确定ndt的分辨率
						oct.genVoxelBoundsFromOctreeKey(it.current_state_->key_, current_level, min_pt, max_pt);
						float res = std::max(std::max((max_pt[0] - min_pt[0]), (max_pt[1] - min_pt[1])), (max_pt[2] - min_pt[2]));
						res_level[current_level - 1] = res;
					}

					level_vec[current_level - 1].push_back(p);
					//auto tt2 = high_resolution_clock::now();
					++it;         //如果是it++，时间要长很久！！！！
					//auto tt3 = high_resolution_clock::now();
					//double dd1 = duration_cast<nanoseconds>(tt2 - tt1).count() / 1e6;
					//double dd2 = duration_cast<nanoseconds>(tt3 - tt2).count() / 1e6;
					//cout << "time creating&pushing points: " << dd1 << " [msecs]." << endl;
					//cout << "time iter ++ outside: " << dd2 << " [msecs]." << endl;

				}

				//auto ts2 = high_resolution_clock::now();
				//double ds = duration_cast<nanoseconds>(ts2 - ts1).count() / 1e9;
				//cout << "points: " << i << endl;
				//cout << "time when reading points: " << ds << " [secs]." << endl;

				vec = level_vec;
				res_vec = res_level;
			}
			void operator()(Octree& oct, std::vector<AlignedPointTVector>& vec, size_t depth)
			{
				OctreeLevelIterator<Octree> it(&oct, 0);
				Eigen::Vector3f min_pt;
				Eigen::Vector3f max_pt;
				min_pt.setZero();
				max_pt.setZero();
				int current_level = 0;

				/*level_vec.resize(oct->getTreeDepth());
				res_level.resize(oct->getTreeDepth());*/
				level_vec.resize(depth);
				res_level.resize(depth);

				//auto ts1 = high_resolution_clock::now();
				//size_t i = 0;
				//while(it.current_state_)
				while ((it.current_state_) && (it.current_state_->depth_ <= depth))
				{
					//i++;
					//auto tt1 = high_resolution_clock::now();
					pcl::PointXYZ p;// (new pcl::PointCloud<pcl::PointXYZ>());
					current_level = it.current_state_->depth_;

					if (current_level == 0)//根节点不存放格子了就
					{
						//auto root_it1 = high_resolution_clock::now();
						++it;
						//auto root_it2 = high_resolution_clock::now();
						//double root_d = duration_cast<nanoseconds>(root_it2 - root_it1).count()/1e6;
						//cout << "root_it++ time: " << root_d << " [msec]." << endl;
						continue;
					}

					oct.genVoxelCenterFromOctreeKey(it.current_state_->key_, current_level, p);

					level_vec[current_level - 1].push_back(p);
					//auto tt2 = high_resolution_clock::now();
					++it;         //如果是it++，时间要长很久！！！！
					//auto tt3 = high_resolution_clock::now();
					//double dd1 = duration_cast<nanoseconds>(tt2 - tt1).count() / 1e6;
					//double dd2 = duration_cast<nanoseconds>(tt3 - tt2).count() / 1e6;
					//cout << "time creating&pushing points: " << dd1 << " [msecs]." << endl;
					//cout << "time iter ++ outside: " << dd2 << " [msecs]." << endl;

				}

				//auto ts2 = high_resolution_clock::now();
				//double ds = duration_cast<nanoseconds>(ts2 - ts1).count() / 1e9;
				//cout << "points: " << i << endl;
				//cout << "time when reading points: " << ds << " [secs]." << endl;

				vec = level_vec;
			}
			void operator()(OctreeLevelContainer<Octree>& str)
			{
				this->operator()((*str.oct), str.vec, str.res_vec);
			}

		public:
			std::vector<AlignedPointTVector> level_vec;
			std::vector<float> res_level;
		};
	}
}

//实现
namespace pcl {
	namespace octree
	{
		//////////////////////////////////////////////////////////////////////////////////////////////
		template <typename OctreeT>
		OctreeLevelIterator<OctreeT>::OctreeLevelIterator(
			unsigned int max_depth_arg)
			: OctreeIteratorBase<OctreeT>(max_depth_arg), FIFO_()
		{
			OctreeIteratorBase<OctreeT>::reset();

			// initialize iterator
			this->reset();
		}

		//////////////////////////////////////////////////////////////////////////////////////////////
		template <typename OctreeT>
		OctreeLevelIterator<OctreeT>::OctreeLevelIterator(
			OctreeT* octree_arg, unsigned int max_depth_arg)
			: OctreeIteratorBase<OctreeT>(octree_arg, max_depth_arg), FIFO_()
		{
			OctreeIteratorBase<OctreeT>::reset();

			// initialize iterator
			this->reset();
		}

		//////////////////////////////////////////////////////////////////////////////////////////////
		template <typename OctreeT>
		void
			OctreeLevelIterator<OctreeT>::reset()
		{
			OctreeIteratorBase<OctreeT>::reset();

			// init FIFO
			FIFO_.clear();

			if (this->octree_) {
				// pushing root node to stack
				//IteratorState FIFO_entry;
				IteratorState* FIFO_entry(new IteratorState());
				FIFO_entry->node_ = this->octree_->getRootNode();
				FIFO_entry->depth_ = 0;
				FIFO_entry->key_.x = FIFO_entry->key_.y = FIFO_entry->key_.z = 0;

				FIFO_.push_back(FIFO_entry);

				this->current_state_ = FIFO_.front();
			}
		}

		//////////////////////////////////////////////////////////////////////////////////////////////
		template <typename OctreeT>
		OctreeLevelIterator<OctreeT>&
			OctreeLevelIterator<OctreeT>::operator++()
		{
			//auto tt1 = high_resolution_clock::now();
			if (FIFO_.size()) {
				// get stack element
				//IteratorState FIFO_entry = *FIFO_.front();      //可采用指针
				

				if ((this->max_octree_depth_ >= ((FIFO_.front())->depth_+1)) &&
					((FIFO_.front())->node_->getNodeType() == BRANCH_NODE)) {
					// current node is a branch node
					BranchNode* current_branch = static_cast<BranchNode*>((FIFO_.front())->node_);

					// iterate over all children
					for (unsigned char child_idx = 0; child_idx < 8; ++child_idx) {

						// if child exist
						if (this->octree_->branchHasChild(*current_branch, child_idx)) {
							
							IteratorState* FIFO_entry(new IteratorState());
							FIFO_entry->depth_ = (FIFO_.front())->depth_ + 1;
							FIFO_entry->key_ = (FIFO_.front())->key_;
							// add child to stack
							OctreeKey& current_key = FIFO_entry->key_;
							current_key.pushBranch(static_cast<unsigned char>(child_idx));

							FIFO_entry->node_ =
								this->octree_->getBranchChildPtr(*current_branch, child_idx);

							FIFO_.push_back(FIFO_entry);

							//current_key.popBranch();
						}
					}
				}

				FIFO_.pop_front();

				if (FIFO_.size()) {
					this->current_state_ = FIFO_.front();
				}
				else {
					this->current_state_ = 0;
				}
			}
			//auto tt2 = high_resolution_clock::now();
			//double dd1 = duration_cast<nanoseconds>(tt2 - tt1).count() / 1e6;
			//cout << "time iter++ inside : " << dd1 << " [msec]." << endl;
			return (*this);
		}
	}
}