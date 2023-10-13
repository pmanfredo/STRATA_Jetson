#include "Segment.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

Segment::Segment() {
    // Constructor (if needed)
}

Segment::~Segment() {
    // Destructor (if needed)
}

void Segment::segmentPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& segmented_cloud) {
    // Define segmentation parameters for objects (customize as needed)
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Set the segmentation method and model type for objects
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);  // Change to the appropriate model type for objects
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);  // Adjust the number of iterations as needed
    seg.setDistanceThreshold(0.01);  // Adjust the distance threshold as needed

    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    // Extract the outliers (objects)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // Set to 'true' to keep objects and exclude the ground

    extract.filter(*segmented_cloud);

    // Print the size of the segmented object cloud
    std::cout << "Segmented object cloud size: " << segmented_cloud->size() << std::endl;
}
