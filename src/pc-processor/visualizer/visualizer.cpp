#include "visualizer.hpp"

#include <pcl/visualization/point_cloud_color_handlers.h>



viz::Visualizer::Visualizer(const std::string &name)
	: viewer(name)
{
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
}

viz::Visualizer::~Visualizer()
{
	viewer.close();
}

void viz::Visualizer::doLoop()
{
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	viewer.close();
}

void viz::Visualizer::addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& id, const std::string& color)
{
	double r, g, b;

	if (color == "hash")
	{
		auto idHash = std::hash<std::string>{}(id);
		r = idHash & 0x7F | 0x80;
		g = (idHash & 0x7F00 | 0x8000) >> 8;
		b = (idHash & 0x7F0000 | 0x800000) >> 16;
	}
	else if (color == "6_0")
	{
		r = 255.0;
		g = 0.0;
		b = 0.0;
	}
	else if (color == "6_1")
	{
		r = 255.0;
		g = 255.0;
		b = 0.0;
	}
	else if (color == "6_2")
	{
		r = 0.0;
		g = 255.0;
		b = 0.0;
	}
	else if (color == "6_3")
	{
		r = 0.0;
		g = 255.0;
		b = 255.0;
	}
	else if (color == "6_4")
	{
		r = 0.0;
		g = 0.0;
		b = 255.0;
	}
	else if (color == "6_5")
	{
		r = 255.0;
		g = 0.0;
		b = 255.0;
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud, r, g, b);
	viewer.addPointCloud(cloud, colorHandler, id);
}


void viz::Visualizer::setCamera(std::array<double, 3> lookAt, std::array<double, 3> pos, std::array<double, 3> up)
{
	pcl::visualization::Camera camera;
	viewer.getCameraParameters(camera);

	std::copy(lookAt.cbegin(), lookAt.cend(), std::begin(camera.focal));
	std::copy(pos.cbegin(), pos.cend(), std::begin(camera.pos));
	std::copy(up.cbegin(), up.cend(), std::begin(camera.view));

	// Set clipping planes
	camera.clip[0] = 0.01; // Close
	camera.clip[1] = 100000.0; // Far

	viewer.setCameraParameters(camera);
}
