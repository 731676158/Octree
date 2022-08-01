#include "fast_vgicp.hpp"
#include "fast_vgicp_impl.hpp"

template class fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>;
