#include "fast_svgicp.hpp"
#include "fast_svgicp_impl.hpp"

template class fast_gicp::FastSVGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastSVGICP<pcl::PointXYZI, pcl::PointXYZI>;
