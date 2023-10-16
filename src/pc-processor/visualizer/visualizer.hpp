#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

#include <string>
#include <array>


namespace viz
{
class Visualizer
{
public:
	Visualizer(const std::string& name = "");
	~Visualizer();

	// Render visualization until exit
	void doLoop();

	// Add an XYZ point cloud to the visualizer
	void addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& id = "", const std::string &color = "hash");

	/// <summary>
	/// Set fundamental camera parameters
	/// </summary>
	/// <param name="lookAt">3D point to look at</param>
	/// <param name="pos">3D position of the camera</param>
	/// <param name="up">3D vector corresponding to "up"</param>
	void setCamera(std::array<double, 3> lookAt, std::array<double, 3> pos, std::array<double, 3> up);


private:
	pcl::visualization::PCLVisualizer viewer;
};
}

#endif VISUALIZER_HPP
