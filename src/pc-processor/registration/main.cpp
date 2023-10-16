#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <vector>
#include <memory>
#include <iostream>

#include "icp_strategy.hpp"
#include "visualizer/visualizer.hpp"


namespace
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadPointClouds(int argc, char* argv[]);
}


int main(int argc, char *argv[])
{
    pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_VERBOSE);

    auto inClouds = loadPointClouds(argc, argv);
    {
        viz::Visualizer visualizer("Individual Point Clouds");
        for (std::size_t i = 0, size = inClouds.size(); i < size; i++)
        {
            visualizer.addPointCloud(inClouds[i], argv[i], std::string("6_") + std::to_string(i));
        }
        visualizer.doLoop();
    }

    auto outClouds = reg::icp_align(inClouds);
    {
        viz::Visualizer visualizer("Stitched Point Cloud");
        for (std::size_t i = 0, size = outClouds.size(); i < size; i++)
        {
            visualizer.addPointCloud(outClouds[i], argv[i], std::string("6_") + std::to_string(i));
        }
        visualizer.doLoop();
    }
}

namespace
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadPointClouds(int argc, char* argv[])
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> inClouds;

        for (int i = 1; i < argc; i++)
        {
            auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            if (pcl::io::loadPCDFile(argv[i], *cloud) != 0)
            {
                continue;
            }

            pcl::Indices indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

            PCL_DEBUG("Loaded point cloud from file %s\n", argv[i]);
            inClouds.emplace_back(cloud);

        }

        return inClouds;
    }
}