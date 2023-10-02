#ifndef PC_LIDAR_PROCESSOR_HPP
#define PC_LIDAR_PROCESSOR_HPP

#include <type_traits>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "scanner_config.hpp"


namespace lidar
{

struct LidarData
{
	double distance; // Millimeters
	double angle; // Radians

#ifdef LIDAR_DATA_HAS_ROTATOR_ANGLE
	double rotatorAngle; // Radians

	LidarData(double distance, double angle, double rotatorAngle) : distance(distance), angle(angle), rotatorAngle(rotatorAngle) {}
#else
	LidarData(double distance, double angle) : distance(distance), angle(angle) {}
#endif

#ifdef LIDAR_DATA_HAS_FLAGS
	friend enum class LidarDataFlag
	{
		ROTATOR_MARKER_0_TURN = 0x00000001,
	};

	friend constexpr LidarDataFlag operator|(LidarDataFlag a, LidarDataFlag b)
	{
		return static_cast<LidarDataFlag>(static_cast<std::underlying_type_t<LidarDataFlag>>(a) | static_cast<std::underlying_type_t<LidarDataFlag>>(b));
	}

	friend constexpr LidarDataFlag operator&(LidarDataFlag a, LidarDataFlag b)
	{
		return static_cast<LidarDataFlag>(static_cast<std::underlying_type_t<LidarDataFlag>>(a) & static_cast<std::underlying_type_t<LidarDataFlag>>(b));
	}

	friend constexpr inline LidarDataFlag operator|=(LidarDataFlag& a, LidarDataFlag b)
	{
		a = a | b;
		return a;
	}


	LidarDataFlag flags;
#endif
};


#ifdef LIDAR_DATA_HAS_FLAGS
// Set flags on the lidar data based on the values
void markFlags(std::vector<LidarData>& lidarData);
#endif


// Convert lidar data into XYZ point clouds oriented at the object's frame of reference
pcl::PointCloud<pcl::PointXYZ>::Ptr dataToPointCloud(const std::vector<LidarData>& lidarData);

} // namespace lidar

#endif // !PC_LIDAR_PROCESSOR_HPP
