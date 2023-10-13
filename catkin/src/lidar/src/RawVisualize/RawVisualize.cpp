#include "RawVisualize.h"

RawVisualize::RawVisualize(const std::string& window_name) {
    viewer_ = boost::make_shared<pcl::visualization::PCLVisualizer>(window_name);
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->addCoordinateSystem(1.0);
}

void RawVisualize::visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    std::cout << "Checking for Empty Point Cloud" << std::endl;
    if (cloud.empty()) {
        std::cerr << "Error: Received an empty point cloud." << std::endl;
        return; // Avoid processing an empty point cloud
    }
    std::cout << "*Point Cloud Not Empty" << std::endl;

    std::cout << "Checking for Viewer" << std::endl;
    if (!viewer_) {
        std::cerr << "Error: Viewer is not properly initialized." << std::endl;
        return; // Avoid viewer-related operations when it's not properly initialized
    }
    std::cout << "*Viewer Okay" << std::endl;

    std::cout << "Removing All Point Clouds" << std::endl;
    viewer_->removeAllPointClouds();
    std::cout << "*Removed All Point Clouds" << std::endl;

    std::cout << "Add Point Cloud" << std::endl;
    viewer_->addPointCloud(cloud.makeShared(), "point_cloud"); // Use makeShared() to create a shared pointer
    std::cout << "*Added Point Cloud" << std::endl;

    std::cout << "Spin" << std::endl;
    //viewer_->spinOnce();
    std::cout << "*Spun" << std::endl;
}
