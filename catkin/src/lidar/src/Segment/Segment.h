#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Segment {
public:
    Segment();
    ~Segment();

    void segmentPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& segmented_cloud);
};
