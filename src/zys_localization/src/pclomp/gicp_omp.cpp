#include <zys_localization/pclomp/gicp_omp.h>
#include <zys_localization/pclomp/gicp_omp_impl.hpp>

template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;

