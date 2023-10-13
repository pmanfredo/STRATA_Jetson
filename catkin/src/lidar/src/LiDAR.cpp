#include "Segment/Segment.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "HPS3DUser_IF.h"
#include "ObjectDetection/ObjectDetection.h"
#include "DownsampleAndFilter/DownsampleAndFilter.h"
#include <thread>
#include <mutex>
#include <condition_variable>

typedef struct
{
    std::vector<HPS3D_PerPointCloudData_t> points;
    uint16_t width;
    uint16_t height;
} PointCloudData_t;

std::vector<pcl::PointCloud<pcl::PointXYZ>> frameBuffer;
std::mutex frameBufferMutex;
std::condition_variable frameBufferNotEmpty;

static HPS3D_MeasureData_t g_measureData;

static void EventCallBackFunc(int handle, int eventType, uint8_t *data, int dataLen, void *userPara);

void processLiDARData(Segment& segment, DownsampleAndFilter& downsample_filter, ObjectDetection& object_detection);

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "LiDAR");
    ros::NodeHandle n;
    ros::Publisher int_pub = n.advertise<std_msgs::Int32>("Chatter", 1000);

    ROS_INFO("Starting...");

    int handle = -1;
    HPS3D_StatusTypeDef ret = HPS3D_RET_OK;

    ret = HPS3D_MeasureDataInit(&g_measureData);
    if (ret != HPS3D_RET_OK)
    {
        ROS_FATAL("MeasureDataInit failed, Err: %d", ret);
        return 0;
    }

    ROS_INFO("MeasureDataInit success!");

    bool lidar_connected = false;

    const char *acm_paths[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"};

    for (const char *acm_path : acm_paths)
    {
        char acm_path_copy[32];
        strncpy(acm_path_copy, acm_path, sizeof(acm_path_copy));
        acm_path_copy[sizeof(acm_path_copy) - 1] = '\0';

        ret = HPS3D_USBConnectDevice(acm_path_copy, &handle);
        if (ret == HPS3D_RET_OK)
        {
            lidar_connected = true;
            break;
        }
    }

    if (!lidar_connected)
    {
        ROS_FATAL("Failed to connect to LiDAR on all ACM devices");
        return 0;
    }

    ret = HPS3D_RegisterEventCallback(EventCallBackFunc, NULL);
    if (ret != HPS3D_RET_OK)
    {
        ROS_FATAL("Register Callback Error: %d", ret);
        return 0;
    }

    HPS3D_DeviceSettings_t settings;
    ret = HPS3D_ExportSettings(handle, &settings);
    if (ret != HPS3D_RET_OK)
    {
        ROS_FATAL("Export Settings Error: %d", ret);
        return 0;
    }

    ROS_INFO("Connected!");
    ROS_INFO("Max resolution: %dX\t%dY", settings.max_resolution_X, settings.max_resolution_Y);

    HPS3D_StartCapture(handle);

    // Create an instance of the Segment class
    Segment segment;

    // Create an instance of ObjectDetection
    ObjectDetection object_detection;
    DownsampleAndFilter downsample_filter;

    std::thread lidarDataThread([&segment, &downsample_filter, &object_detection]() {
        processLiDARData(segment, downsample_filter, object_detection);
    });

    ros::Rate loop_rate(40);
    int count = 0;
    while (ros::ok())
    {
        std_msgs::Int32 msg;
        msg.data = count;
        int_pub.publish(msg);
        ros::spinOnce();
        count++;
        loop_rate.sleep();
    }

    lidarDataThread.join();

    ROS_INFO("Shutting down...");
    HPS3D_StopCapture(handle);
    HPS3D_CloseDevice(handle);

    return 0;
}

static void EventCallBackFunc(int handle, int eventType, uint8_t *data, int dataLen, void *userPara)
{
    pcl::PointCloud<pcl::PointXYZ> point_data;
    HPS3D_PerPointCloudData_t *dataptr;

    switch ((HPS3D_EventType_t)eventType)
    {
        case HPS3D_FULL_DEPTH_EVEN:
            HPS3D_ConvertToMeasureData(data, &g_measureData, (HPS3D_EventType_t)eventType);

            dataptr = g_measureData.full_depth_data.point_cloud_data.point_data;
            for (int i = 0; i < g_measureData.full_depth_data.point_cloud_data.points; i++)
            {
                point_data.push_back(pcl::PointXYZ(dataptr[i].x, dataptr[i].y, dataptr[i].z));
            }
            point_data.width = g_measureData.full_depth_data.point_cloud_data.width;
            point_data.height = g_measureData.full_depth_data.point_cloud_data.height;

            ROS_INFO("Received Point Cloud Data: Size = %zu", point_data.size());

            {
                std::lock_guard<std::mutex> lock(frameBufferMutex);
                frameBuffer.push_back(point_data);
                frameBufferNotEmpty.notify_all();
            }

            break;

        case HPS3D_SIMPLE_ROI_EVEN:
        case HPS3D_FULL_ROI_EVEN:
        case HPS3D_SIMPLE_DEPTH_EVEN:
            ROS_WARN("Wrong data");
            break;

        case HPS3D_SYS_EXCEPTION_EVEN:
            ROS_WARN("SYS ERR :%s", data);
            break;

        case HPS3D_DISCONNECT_EVEN:
            ROS_ERROR("Device disconnected!");
            HPS3D_StopCapture(handle);
            break;
    }
}

void processLiDARData(Segment& segment, DownsampleAndFilter& downsample_filter, ObjectDetection& object_detection)
{
    ros::Rate loop_rate(40);
    while (ros::ok())
    {
        {
            std::unique_lock<std::mutex> lock(frameBufferMutex);
            frameBufferNotEmpty.wait(lock, [] { return !frameBuffer.empty(); });

            pcl::PointCloud<pcl::PointXYZ> frame = frameBuffer.front();
            lock.unlock();

            ROS_INFO("Frame Size Before Downsampling: %zu", frame.size());

            // Create a pointer to the frame and pass it to the segment function
            pcl::PointCloud<pcl::PointXYZ>::Ptr frame_ptr(new pcl::PointCloud<pcl::PointXYZ>(frame));
            pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            segment.segmentPointCloud(frame_ptr, segmented_cloud);

            // Object detection logic can be added here, using the segmented_cloud
			object_detection.detectObjects(segmented_cloud);
		}

        loop_rate.sleep();
    }
}
