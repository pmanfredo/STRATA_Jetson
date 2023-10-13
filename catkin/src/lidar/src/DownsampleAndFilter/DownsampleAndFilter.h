#ifndef DOWNSAMPLE_AND_FILTER_H
#define DOWNSAMPLE_AND_FILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DownsampleAndFilter {
public:
    DownsampleAndFilter();

    // Function to downsample point cloud data with an adjustable leaf size
    void downsample(const pcl::PointCloud<pcl::PointXYZ>& inputCloud,
                             pcl::PointCloud<pcl::PointXYZ>& downsampledCloud);

    // Function to remove outliers from the point cloud
    void removeOutliers(const pcl::PointCloud<pcl::PointXYZ>& inputCloud,
                        pcl::PointCloud<pcl::PointXYZ>& filteredCloud);

    // Function to set the leaf size for downsampling
    void setLeafSize(float leaf_size);

private:
    float leaf_size_; // Private member variable to store the leaf size
};

#endif // DOWNSAMPLE_AND_FILTER_H
