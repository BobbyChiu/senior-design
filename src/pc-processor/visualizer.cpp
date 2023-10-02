#include "visualizer.hpp"



viz::Visualizer::Visualizer(const std::string &name)
	: viewer(name)
{
	viewer.addCoordinateSystem(10.0);
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

void viz::Visualizer::addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string &id)
{
	viewer.addPointCloud(cloud, id);
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
