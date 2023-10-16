#ifndef PC_REGISTRATION_ICP_HPP
#define PC_REGISTRATION_ICP_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace reg
{
// Pairwise stitch using iterative closest point, using the first point cloud as the target.
// Caller receives ownership of the returned pointer.
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> icp_align(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pointClouds);
}


#endif // PC_REGISTRATION_ICP_HPP