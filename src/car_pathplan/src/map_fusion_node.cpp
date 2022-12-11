
#include "../include/map_fusion/class_map_fusion.h"

#include "ros/ros.h"
// #include "tf/transform_listener.h"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <algorithm>
// #include <atomic>
// #include <thread>
// #include <boost/property_tree/ptree.hpp>
// #include <boost/property_tree/json_parser.hpp>

// #include "sensor_msgs/LaserScan.h"
// #include "nav_msgs/OccupancyGrid.h"
// #include "nav_msgs/MapMetaData.h"
// #include "nav_msgs/Odometry.h"
// #include "nav_msgs/Path.h"
// #include "geometry_msgs/PoseStamped.h"





int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_fusion_node");

    ros::NodeHandle n;

    ClassMapFusion map_fusor( n );

    // ros::spin();
    ros::AsyncSpinner s(2);
    s.start();

    ros::waitForShutdown();

    return 0;
}





