#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <chrono>
#include <thread>

#include <Eigen/core>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include "scanner_config.hpp"
#include "leveler/leveler.hpp"
#include "leveler/simulated_leveler.hpp"
#include "visualizer.hpp"


namespace
{
struct SystemRawData
{
    struct Lidar
    {
        double r;
        double theta;
        Lidar(double r, double theta) : r(r), theta(theta) {}
    } lidar;
    struct Rotator
    {
        double theta;
        Rotator(double theta) : theta(theta) {}
    } rotator;

public:
    SystemRawData(double lidarR, double lidarTheta, double rotatorTheta)
        : lidar(lidarR, lidarTheta), rotator(rotatorTheta)
    {}
};


static pcl::PointCloud<pcl::PointXYZ>::Ptr lidarToPointCloudTest()
{
    // Let lidar reference be at (3, 4), leveler reference be at x=7
    // heightDelta = 4
    // horizontalDelta = 7 - 3 = 4

    // List of points represented as distance and angle
    // const std::vector<SystemRawData> systemRawData{
    //     {4000.0_mm, 0.0_deg, 0.0_deg}, // +(4, 0) => (7, 4) => (0, 0, 4)
    //     {5000.0_mm, 36.870_deg, 0.0_deg}, // +(4, 3) => (7, 7) => (0, 0, 7)
    //     {3162.5_mm, 341.565_deg, 0.0_deg}, // +(3, -1) => (6, 3) => (1, 0, 3)

    //     {4000.0_mm, 0.0_deg, 90.0_deg},
    //     {5000.0_mm, 36.870_deg, 90.0_deg},
    //     {3162.5_mm, 341.565_deg, 90.0_deg},

    //     {4000.0_mm, 0.0_deg, 180.0_deg},
    //     {5000.0_mm, 36.870_deg, 180.0_deg},
    //     {3162.5_mm, 341.565_deg, 180.0_deg},

    //     {4000.0_mm, 0.0_deg, 270.0_deg},
    //     {5000.0_mm, 36.870_deg, 270.0_deg},
    //     {3162.5_mm, 341.565_deg, 270.0_deg},
    // };

    std::vector<SystemRawData> systemRawData;
    constexpr auto SECTOR_COUNT = 40;
    for (std::size_t i = 0; i < SECTOR_COUNT; i++)
    {
        constexpr auto START_ANGLE = -45.0_deg;
        constexpr auto END_ANGLE = 60.0_deg;
        constexpr auto DELTA_ANGLE = 4.319E-2;

        for (auto angle = START_ANGLE; angle < END_ANGLE; angle += DELTA_ANGLE)
        {
            systemRawData.emplace_back(3000.0_mm, angle, i * 2 * M_PI / SECTOR_COUNT);
        }
    }

    pcl::PointCloud<pcl::PointXY> lidarCartesian;
    lidarCartesian.width = systemRawData.size();
    lidarCartesian.height = 1;
    lidarCartesian.resize(lidarCartesian.width * lidarCartesian.height);
    for (std::size_t i = 0; i < lidarCartesian.size(); i++)
    {
        const auto& lidarPolar = systemRawData[i].lidar;
        lidarCartesian[i].x = lidarPolar.r * std::cos(lidarPolar.theta);
        lidarCartesian[i].y = lidarPolar.r * std::sin(lidarPolar.theta);
    }

    Eigen::Affine2f lidarToRotatorTransform = Eigen::Affine2f::Identity();
    lidarToRotatorTransform.scale(Eigen::Vector2f(-1, +1));
    lidarToRotatorTransform.translation() << config::horizontalDelta, config::heightDelta;

    pcl::PointCloud<pcl::PointXY> rotatorCartesian;
    pcl::transformPointCloud(lidarCartesian, rotatorCartesian, lidarToRotatorTransform);

    auto objectPoints = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    objectPoints->width = rotatorCartesian.size();
    objectPoints->height = 1;
    objectPoints->resize(objectPoints->width * objectPoints->height);
    std::cout << "Listing points in object reference:" << '\n';
    for (std::size_t i = 0; i < objectPoints->size(); i++)
    {
        auto objectR = rotatorCartesian[i].x;
        auto objectTheta = systemRawData[i].rotator.theta;
        auto objectZ = rotatorCartesian[i].y;
        (*objectPoints)[i].x = objectR * std::cos(objectTheta);
        (*objectPoints)[i].y = objectR * -std::sin(objectTheta);
        (*objectPoints)[i].z = objectZ;

        if (objectPoints->size() < 50)
        {
            std::cout << (*objectPoints)[i].x << ", " << (*objectPoints)[i].y << ", " << (*objectPoints)[i].z << '\n';
        }
    }

    return objectPoints;
}

static void levelerInterfaceTest()
{
    std::cout << "Polling leveler every 0.1s:" << '\n';
    std::unique_ptr<Leveler> leveler = std::make_unique<SimulatedLeveler>(-2.0_rad, 3.0_rad / 1.0);

    for (int i = 0; i < 10; i++)
    {
        using namespace std::chrono_literals;

        std::cout << leveler->pollAngle() << '\n';

        std::this_thread::sleep_for(0.1s);
    }
}
}


int main(int argc, char* argv[])
{
    auto cloud = lidarToPointCloudTest();

    viz::Visualizer viewer("Test Visualizer");
    viewer.addPointCloud(cloud);


    viewer.setCamera({ 0, 0, 4 }, { 3, 0, 4 }, { 0, 0, 1 });
    viewer.doLoop();


    levelerInterfaceTest();

    return 0;
}
