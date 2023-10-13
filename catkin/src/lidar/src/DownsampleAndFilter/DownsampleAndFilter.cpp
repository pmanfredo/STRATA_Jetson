#include "DownsampleAndFilter.h"
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

DownsampleAndFilter::DownsampleAndFilter() {
    // Constructor
    leaf_size_ = 1.0f; // Default leaf size
}

void DownsampleAndFilter::setLeafSize(float leaf_size) {
    leaf_size_ = leaf_size;
}

void DownsampleAndFilter::downsample(const pcl::PointCloud<pcl::PointXYZ>& inputCloud,
                                     pcl::PointCloud<pcl::PointXYZ>& downsampledCloud) {
    // Voxel grid downsampling
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(inputCloud.makeShared());
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_); // Use the leaf_size_ member variable
    sor.filter(downsampledCloud);

    // Debug print statement
    std::cout << "Downsampled Point Cloud Size: " << downsampledCloud.size() << std::endl;
}

void DownsampleAndFilter::removeOutliers(const pcl::PointCloud<pcl::PointXYZ>& inputCloud,
                                        pcl::PointCloud<pcl::PointXYZ>& filteredCloud) {
    // Create a KdTreeFLANN for the input cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(inputCloud.makeShared());

    // Define the search radius (e.g., 5 centimeters)
    const float searchRadius = 0.10; // 10 centimeters in meters

    // Iterate through the points in the input cloud and remove outliers using radius-based search
    for (const pcl::PointXYZ& point : inputCloud.points) {
        std::vector<int> pointIdx;
        std::vector<float> pointSquaredDistance;

        // Perform radius-based search with the chosen search radius
        if (kdtree.radiusSearch(point, searchRadius, pointIdx, pointSquaredDistance) > 1) {
            // If there are more than one point within the specified radius, consider it an inlier
            filteredCloud.push_back(point);
        }else {
            // Debug print statement
            //std::cout << "Point at (" << point.x << ", " << point.y << ", " << point.z << ") is an outlier." << std::endl;
        }
    }

    // Debug print statement
    std::cout << "Filtered Point Cloud Size: " << filteredCloud.size() << std::endl;
}
