#ifndef RAW_VISUALIZE_H
#define RAW_VISUALIZE_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

class RawVisualize {
public:
    RawVisualize(const std::string& window_name = "PointCloud Visualization");
    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};

#endif // RAW_VISUALIZE_H
