#include <zys_localization/pclomp/ndt_omp.h>
#include <zys_localization/pclomp/ndt_omp_impl.hpp>

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
