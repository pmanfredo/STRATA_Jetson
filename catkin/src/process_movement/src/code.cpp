#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "process_movement");
    ros::NodeHandle nh;

    ROS_INFO("process_movement node is running.");

    // processing movement goes here

    ros::spin();

    return 0;
}
