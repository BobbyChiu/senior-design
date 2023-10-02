#include "lidar_processor.hpp"

#include <pcl/common/transforms.h>


#ifdef LIDAR_DATA_HAS_FLAGS
void lidar::markFlags(std::vector<lidar::LidarData>& lidarData)
{
	// TODO: Detect and set markers
    std::size_t i, size = lidarData.size();
    signed int lastFlag;
    std::size_t startOfValid, endOfValid; // Keep all samples between startOfValid (inclusive) and endOfValid (exclusive)

	// For now, mark every 80000 samples as the start of the "0" marker
	for (i = 0; i < size; i += 80000)
	{
		lidarData[i].flags |= LidarData::LidarDataFlag::ROTATOR_MARKER_0_TURN;
	}

    // Calculate rotator angles based on markers
    for (i = 0, lastFlag = -1; i < size; i++)
    {
        if ((lidarData[i].flags & LidarData::LidarDataFlag::ROTATOR_MARKER_0_TURN) == LidarData::LidarDataFlag::ROTATOR_MARKER_0_TURN)
        {
            if (lastFlag == -1)
            {
                lastFlag = i;
                startOfValid = i;
            }
            else
            {
                std::size_t samples = i - lastFlag;
                for (std::size_t j = 0; j < samples; j++)
                {
                    lidarData[j + lastFlag].rotatorAngle = static_cast<double>(j) / samples * 360.0;
                }
            }
        }
    }
    endOfValid = lastFlag;

    // Keep only valid samples
    lidarData.erase(lidarData.cbegin() + endOfValid, lidarData.cend());
    lidarData.erase(lidarData.cbegin(), lidarData.cbegin() + startOfValid);
}
#endif


pcl::PointCloud<pcl::PointXYZ>::Ptr lidar::dataToPointCloud(const std::vector<lidar::LidarData>& lidarData)
{
    // Convert polar to cartesian
    pcl::PointCloud<pcl::PointXY> lidarCartesian;
    lidarCartesian.width = lidarData.size();
    lidarCartesian.height = 1;
    lidarCartesian.resize(lidarCartesian.width * lidarCartesian.height);
    for (std::size_t i = 0; i < lidarCartesian.size(); i++)
    {
        const auto& lidarPolar = lidarData[i];
        lidarCartesian[i].x = lidarPolar.distance * std::cos(lidarPolar.angle);
        lidarCartesian[i].y = lidarPolar.distance * std::sin(lidarPolar.angle);
    }

    // Transform frame of reference from lidar to rotator
    Eigen::Affine2f lidarToRotatorTransform = Eigen::Affine2f::Identity();
    lidarToRotatorTransform.scale(Eigen::Vector2f(-1, +1));
    lidarToRotatorTransform.translation() << config::horizontalDelta, config::heightDelta;

    pcl::PointCloud<pcl::PointXY> rotatorCartesian;
    pcl::transformPointCloud(lidarCartesian, rotatorCartesian, lidarToRotatorTransform);

    // Apply rotator angle to obtain object cartesian
    auto objectPoints = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    objectPoints->width = rotatorCartesian.size();
    objectPoints->height = 1;
    objectPoints->resize(objectPoints->width * objectPoints->height);
    for (std::size_t i = 0; i < objectPoints->size(); i++)
    {
        auto objectR = rotatorCartesian[i].x;
        auto objectTheta = lidarData[i].rotatorAngle;
        auto objectZ = rotatorCartesian[i].y;
        (*objectPoints)[i].x = objectR * std::cos(objectTheta);
        (*objectPoints)[i].y = objectR * -std::sin(objectTheta);
        (*objectPoints)[i].z = objectZ;
    }

    return objectPoints;
}
