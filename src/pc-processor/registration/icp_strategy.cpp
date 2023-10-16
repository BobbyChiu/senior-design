#include "icp_strategy.hpp"

#include <pcl/registration/icp_nl.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <memory>
#include <iostream>


namespace {

class PointNormalRep : public pcl::PointRepresentation <pcl::PointNormal>
{
public:
    PointNormalRep()
    {
        pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_ = 4;
    }

    virtual void copyToFloatArray (const pcl::PointNormal &p, float *out) const
    {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

    Eigen::Matrix4f calculateTransform(const pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud)
    {
        auto filteredSource = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        auto filteredTarget = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setLeafSize(0.05, 0.05, 0.05);

        grid.setInputCloud(sourceCloud);
        grid.filter(*filteredSource);

        grid.setInputCloud(targetCloud);
        grid.filter(*filteredTarget);

        auto sourceN = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        auto targetN = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normEstimator;
        auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
        normEstimator.setSearchMethod(tree);
        normEstimator.setKSearch(30);

        normEstimator.setInputCloud(filteredSource);
        normEstimator.compute(*sourceN);
        pcl::copyPointCloud(*filteredSource, *sourceN);

        normEstimator.setInputCloud(filteredTarget);
        normEstimator.compute(*targetN);
        pcl::copyPointCloud(*filteredTarget, *targetN);

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;

        auto pointRep = std::make_shared<PointNormalRep>();
        float rescaleValues[4] = {1.0, 1.0, 1.0, 1.0};
        pointRep->setRescaleValues(rescaleValues);

        icp.setTransformationEpsilon(1e-6);
        double maxCorrespondenceDist = 0.1;
        int numberIterationsRemaining = 60;
        for (; numberIterationsRemaining > 0 && maxCorrespondenceDist > 0; maxCorrespondenceDist -= 0.001)
        {
            icp.setMaxCorrespondenceDistance(maxCorrespondenceDist);
            icp.setMaximumIterations(numberIterationsRemaining);
            icp.setPointRepresentation(pointRep);

            icp.setInputSource(sourceN);
            icp.setInputTarget(targetN);

            auto alignedSourceN = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
            icp.align(*alignedSourceN);
            sourceN = alignedSourceN;

            transform = icp.getFinalTransformation() * transform;

            numberIterationsRemaining -= icp.nr_iterations_;
        }

        return transform;
}
}


std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> reg::icp_align(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &pointClouds)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> outClouds;

    std::size_t size = pointClouds.size();
    if (size < 1)
    {
        return outClouds;
    }

    outClouds.emplace_back(pointClouds[0]);

    Eigen::Matrix4f accumulatedTransform = Eigen::Matrix4f::Identity();
    for (std::size_t i = 1; i < size; i++)
    {
        auto transform = calculateTransform(pointClouds[i], pointClouds[i - 1]);
        accumulatedTransform = transform * accumulatedTransform;
        std::cout << accumulatedTransform << '\n';

        auto transformedSource = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(*pointClouds[i], *transformedSource, accumulatedTransform);

        outClouds.emplace_back(transformedSource);
    }

    return outClouds;
}
