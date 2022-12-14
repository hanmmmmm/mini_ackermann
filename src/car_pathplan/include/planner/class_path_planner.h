#ifndef CLASS_PATH_PLANNER
#define CLASS_PATH_PLANNER

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <mutex>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"

// #include "../hybrid_a_star_module/hybrid_astar.h"
#include "../hybrid_a_star_module/hybrid_astar_2.h"

using std::chrono::high_resolution_clock;

/*
The pose of start/robot, goal, path are converted from /map frame to map-grid-space;
converted_tf = tf_in_map_frame - map_msg_origin;

*/


/*
TODO:
add a function to check if the current path is still valid:
- robot position xy is close to it
- robot heading is close to the path
- path is clear from obstacles
- 
if it's still valid, then no need to search new path. 

*/


class ClassPathPlanner
{
private:

    struct robotPose
    {
        double x_meter, y_meter; // meter
        double robot_yaw_rad;    // radian
        int x_grid, y_grid;
        int x_grid_prev, y_grid_prev;
    };    

    Hybrid_astar_class planner_;

    int path_plan_timeout_ms_;

    std::array<double, 3> start_pose_;
    std::array<double, 3> goal_pose_;

    // std::string odom_topic_name_; 
    std::string path_published_topic_name_; 
    std::string map_subscribed_topic_name_; 
    std::string goal_subscribed_topic_name_;

    ros::Publisher path_puber_ ;
    // ros::Subscriber odom_suber_ ;
    ros::Subscriber map_suber_ ;
    ros::Subscriber goal_suber_;

    bool map_received_, goal_received_; 

    nav_msgs::Path path_msg_;

    ros::Timer  periodic_path_planer_;
    double planer_interval_;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;

    std::mutex map_mutex_, odom_mutex_, goal_mutex_;

    std::string map_frame_, robot_frame_; //, depth_frame_, sonic_frame_; 

    void load_parameters();

    double mod_2pi(double angle);

    nav_msgs::OccupancyGrid map_msg_;
    geometry_msgs::PoseStamped goal_msg_;

    tf::StampedTransform robot_to_map_tf_;

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);


    void path_plan( const ros::TimerEvent &event );

    void get_robot_pose_in_map_frame();

    double helper_get_time();


public:
    ClassPathPlanner(const ros::NodeHandle nh_in_);
    ~ClassPathPlanner();
};

ClassPathPlanner::ClassPathPlanner(const ros::NodeHandle nh_in_): nh_{nh_in_}
{
    load_parameters(); 

    map_received_ = false;
    goal_received_ = false; 

    path_puber_  = nh_.advertise<nav_msgs::Path>( path_published_topic_name_, 10);

    map_suber_ = nh_.subscribe( map_subscribed_topic_name_ , 1, &ClassPathPlanner::map_callback,  this);
    goal_suber_= nh_.subscribe( goal_subscribed_topic_name_ , 1, &ClassPathPlanner::goal_callback, this);


    periodic_path_planer_ = nh_.createTimer( ros::Duration(planer_interval_), &ClassPathPlanner::path_plan, this );

    start_pose_ = {0,0,0};
    goal_pose_ = {0,0,0};
    std::cout << "ClassPathPlanner inti Done" << std::endl;

}

ClassPathPlanner::~ClassPathPlanner()
{
}




void ClassPathPlanner::load_parameters(){
    map_subscribed_topic_name_ = "/map_fusion";
    goal_subscribed_topic_name_ = "/move_base_simple/goal";
    path_published_topic_name_ = "/path";

    map_frame_ = "/map";
    robot_frame_ = "/base_link";

    planer_interval_ = 0.1;
}


void ClassPathPlanner::path_plan( const ros::TimerEvent &event ){

    if( ! map_received_ ) return;
    if( ! goal_received_ ) return;

    // std::cout << "\npath_plan start" << std::endl;

    double t1 = helper_get_time();

    map_mutex_.lock();
    goal_mutex_.lock();

    get_robot_pose_in_map_frame();

    planner_.setup(2000, start_pose_, goal_pose_, map_msg_.info.width, map_msg_.info.height, map_msg_.data, map_msg_.info.resolution);

    // planner_.search();

    

    bool found_path = planner_.search();

    std::cout << "Search Done.  found_path:" << found_path << std::endl;

    if( found_path ){
        // std::cout << "in if" << std::endl;
        std::deque< array<double, 3> >  path;
        planner_.get_path(path);

        // std::cout << "path size: " << path.size() << std::endl;

        path_msg_.header.frame_id = map_frame_;

        path_msg_.poses.clear();
        geometry_msgs::PoseStamped one_pose;
        tf2::Quaternion myQuaternion;

        for( auto point : path ){
            // std::cout << "in for" << std::endl;
            one_pose.pose.position.x = point[0] + map_msg_.info.origin.position.x;
            one_pose.pose.position.y = point[1] + map_msg_.info.origin.position.y;
            one_pose.pose.position.z = 0.0;

            myQuaternion.setRPY(0,0,point[2]);
            myQuaternion=myQuaternion.normalize();
            one_pose.pose.orientation.w = myQuaternion.w();
            one_pose.pose.orientation.x = myQuaternion.x();
            one_pose.pose.orientation.y = myQuaternion.y();
            one_pose.pose.orientation.z = myQuaternion.z();

            path_msg_.poses.push_back(one_pose);
            // std::cout << "path_ " << point[0] << " " << point[1] << " " << std::endl;
        }

        path_puber_.publish( path_msg_ );
    }

    // std::cout << "after if" << std::endl;

    map_mutex_.unlock();
    goal_mutex_.unlock(); 

    double t2 = helper_get_time(); 

    std::cout << "function path_plan " << int((t2-t1)*1000.0) << " ms" << std::endl;

}





void ClassPathPlanner::get_robot_pose_in_map_frame(){
    try{
        tf_listener.lookupTransform( map_frame_, robot_frame_, ros::Time(0), robot_to_map_tf_);
        start_pose_[0] = robot_to_map_tf_.getOrigin().x() - map_msg_.info.origin.position.x;
        start_pose_[1] = robot_to_map_tf_.getOrigin().y() - map_msg_.info.origin.position.y;
        // start_pose_[2] = robot_to_map_tf_.getRotation().

        tf::Quaternion q(robot_to_map_tf_.getRotation().x(),
                         robot_to_map_tf_.getRotation().y(),
                         robot_to_map_tf_.getRotation().z(),
                         robot_to_map_tf_.getRotation().w()  );

        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(q);
        mat.getEulerYPR(yaw, pitch, roll);
        start_pose_[2] = mod_2pi(yaw);
        // std::cout << start_pose_[0] << " " << start_pose_[1] << " " << start_pose_[2] << std::endl;
        // std::cout << "robot yaw  " << start_pose_[2]*180.0/M_PI << std::endl;

    }
    catch (tf::TransformException ex){
      ROS_ERROR("\nget_robot_pose_in_map_frame: \n%s",ex.what());
    //   ros::Duration(0.1).sleep();
    }
}


void ClassPathPlanner::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_mutex_.lock();
    // std::cout << "map_callback start" << std::endl;
    map_msg_.data = msg->data;
    map_msg_.header = msg->header;
    map_msg_.info = msg->info;
    map_received_ = true;
    // std::cout << "map_callback end" << std::endl;
    map_mutex_.unlock();
}


void ClassPathPlanner::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_mutex_.lock();

    goal_msg_ = *msg;

    goal_pose_[0] = msg->pose.position.x - map_msg_.info.origin.position.x;
    goal_pose_[1] = msg->pose.position.y - map_msg_.info.origin.position.y;
    tf::Quaternion q(msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w  );
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);

    goal_pose_[2] =  mod_2pi(yaw);

    goal_received_ = true;
    
    goal_mutex_.unlock();

}


double ClassPathPlanner::mod_2pi( double a)
{
    double angle = a;
    while (angle > 2*M_PI){
        angle -= 2*M_PI;
    }
    while (angle < 0){
        angle += 2*M_PI;
    }
    return angle;
}





/**
 * @brief 
 * return time in seconds, epoch time. 
 */
double ClassPathPlanner::helper_get_time()
{
    return high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}












#endif
