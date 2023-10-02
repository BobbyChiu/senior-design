#include "lidar_processor.hpp"
#include "visualizer.hpp"


int main(int argc, char* argv[])
{
    // Let lidar reference be at (3, 4), leveler reference be at x=7
    // heightDelta = 4
    // horizontalDelta = 7 - 3 = 4

    // List of points represented as distance and angle
    const std::vector<lidar::LidarData> lidarData{
        {4000.0, 0.0, 0.0}, // +(4, 0) => (7, 4) => (0, 0, 4)
        {5000.0, DEG2RAD(36.870), 0.0}, // +(4, 3) => (7, 7) => (0, 0, 7)
        {3162.5, DEG2RAD(341.565), 0.0}, // +(3, -1) => (6, 3) => (1, 0, 3)

        {4000.0, 0.0, DEG2RAD(90.0)},
        {5000.0, DEG2RAD(36.870), DEG2RAD(90.0)},
        {3162.5, DEG2RAD(341.565), DEG2RAD(90.0)},

        {4000.0, 0.0, DEG2RAD(180.0)},
        {5000.0, DEG2RAD(36.870), DEG2RAD(180.0)},
        {3162.5, DEG2RAD(341.565), DEG2RAD(180.0)},

        {4000.0, 0.0, DEG2RAD(270.0)},
        {5000.0, DEG2RAD(36.870), DEG2RAD(270.0)},
        {3162.5, DEG2RAD(341.565), DEG2RAD(270.0)},
    };

	// std::vector<lidar::LidarData> lidarData;
    // constexpr auto SECTOR_COUNT = 40;
    // for (std::size_t i = 0; i < SECTOR_COUNT; i++)
    // {
    //     constexpr auto START_ANGLE = DEG2RAD(-45.0);
    //     constexpr auto END_ANGLE = DEG2RAD(60.0);
    //     constexpr auto DELTA_ANGLE = 4.319E-2;

    //     for (auto angle = START_ANGLE; angle < END_ANGLE; angle += DELTA_ANGLE)
    //     {
    //         lidarData.emplace_back(3000.0, angle, i * 2 * M_PI / SECTOR_COUNT);
    //     }
    // }


	// lidar::markFlags(lidarData);
	auto pointCloud = lidar::dataToPointCloud(lidarData);

    viz::Visualizer viewer("Test Visualizer");
    viewer.addPointCloud(pointCloud);


    viewer.setCamera({ 0, 0, 4000 }, { 3000, 0, 4000 }, { 0, 0, 1 });
    viewer.doLoop();
}
