
#ifndef HYBRID_ASTAR_H
#define HYBRID_ASTAR_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <math.h>

#include <algorithm>
#include <set>
#include <chrono>

#include "array_hasher.h"

#include "enum_fail_reasons.h"

#include "Reeds_Shepp_curve/reedsshepp.h"

using std::array;
using std::string;
using std::vector;
using std::cout;
using std::endl;


using std::chrono::high_resolution_clock;

enum EnumExploreState : char
{
    NEW = 0,
    OPEN = 1,
    CLOSED = 2
};


// it might be better to split grids into openlist and closedlist, for reduce time for searching the min-cost grids. 


class Hybrid_astar_class  
{
private:

    ReedsSheppClass RS_curve_finder_;
    int counter_for_rs_search_;
    int interval_for_rs_search_;

    vector<int8_t> *grid_map_;

    int grid_map_width_, grid_map_height_;

    int8_t obstacle_threshold_value_;

    double fine_to_grid_ratio_ = 0.0;
    const double angle_resolution_ = M_PI/18.0; // radian

    int time_out_ms_;

    vector< vector<double> > motion_model_;

    array<int,   3> start_grid_;
    array<double, 3> start_pose_;

    array<int,   3> goal_grid_;
    array<double, 3> goal_pose_;

    array<double, 3> close_goal_pose_;

    std::array<int, 3> start_grid_for_rs_curve_;

    double close_goal_angle_; 

    struct gridInfo
    {
        double gcost = 0;
        double fcost = 0;
        // EnumExploreState state = EnumExploreState::NEW;
        int steer_type = 0;  // 0-5, total 6 types
        array<double, 3> fine_pose;
        array<int, 3> parent;
    };
    
    // std::unordered_map<array<int, 3>, gridInfo , ArrayHasher3> all_grids_;
    std::unordered_map<array<int, 3>, gridInfo , ArrayHasher3> open_grids_;
    std::unordered_map<array<int, 3>, gridInfo , ArrayHasher3> clos_grids_;

    struct goalTolerance
    {
       double max_distance_error;
       double max_heading_error;
    };

    goalTolerance goal_tolerance;
   
    double step_length_ ;
    double turning_raius_  ;
    double turning_angle_ ;

    bool FLAG_reach_goal_ ;
    bool FLAG_trapped_;
    bool FLAG_inside_obstacle_;
    bool FLAG_found_rs_solution_;

    EnumFailReasons fail_reasons_;



    std::deque< array<double, 3> > path_;
    std::vector< array<double, 3>> rs_path_;
    

    void build_motion_model();

    void print_init_info();

    double mod_2pi(double angle);

    // int improve_hcost( array<double, 3> node_pose, int hcost, int scan_range, bool same_steer);

    void check_if_reach_goal( array<double, 3> node_pose, array<double, 3> in_goal_pose, array<int, 3> in_curr_grid);

    int twoD_to_oneD(const int x, const int y, const int width, const int height);

    array<int,3> helper_convert_fine_pose_to_grid(const array<double, 3> fine_pose , double dist_ratio, double ang_ratio );

    void explore_one_node(array<double, 3> curr_pose, array<int, 3> curr_grid);
    
    void explore_one_ite(vector< array<int, 3>> active_nodes);

    vector< array<int,3>> find_min_cost_nodes();

    double compute_h_cost_Euclidean(const array<double, 3> n, const array<double, 3> g);
    double compute_h_cost_Manhattan(const array<double, 3> n, const array<double, 3> g);
    double compute_h_cost_Chebyshev(const array<double, 3> n, const array<double, 3> g);


public:
    Hybrid_astar_class();
    ~Hybrid_astar_class();

    void setup(const int timeout_ms, 
                const array<double,3> startpose, 
                const array<double,3> goalpose ,
                const int map_width_grid, 
                const int map_height_grid, 
                vector<int8_t>& map,
                const double grid_resolution);

    bool search();

    void get_path( std::deque< array<double, 3> >& path);

};



Hybrid_astar_class::Hybrid_astar_class() 
{
}

Hybrid_astar_class::~Hybrid_astar_class() 
{
}


void Hybrid_astar_class::setup(
                                const int timeout_ms,
                                const array<double,3> startpose, 
                                const array<double,3> goalpose,
                                const int map_width_grid, 
                                const int map_height_grid, 
                                vector<int8_t>& map,
                                const double grid_resolution 
                                )
{

    std::cout << "Hybrid_astar_class::setup() START" << std::endl;

    time_out_ms_ = timeout_ms;

    grid_map_ = &map;

    start_pose_ = startpose;
    goal_pose_  = goalpose ;

    fine_to_grid_ratio_ = grid_resolution ;

    start_grid_ = helper_convert_fine_pose_to_grid(start_pose_, fine_to_grid_ratio_, angle_resolution_);
    goal_grid_  = helper_convert_fine_pose_to_grid(goal_pose_,  fine_to_grid_ratio_, angle_resolution_);

    

    // std::cout << "start_grid_ " << std::endl;
    // std::cout << start_grid_[0] << " " << start_grid_[1] << " " << start_grid_[2] << std::endl;
    // std::cout << start_pose_[0] << " " << start_pose_[1] << " " << start_pose_[2] << std::endl;

    // std::cout << "goal_grid_ " << std::endl;
    // std::cout << goal_grid_[0] << " " << goal_grid_[1] << " " << goal_grid_[2] << std::endl;
    // std::cout << goal_pose_[0] << " " << goal_pose_[1] << " " << goal_pose_[2] << std::endl;

    path_.clear();
    rs_path_.clear(); 
    // all_grids_.clear();
    open_grids_.clear();
    clos_grids_.clear();

    FLAG_reach_goal_ = false;
    FLAG_trapped_ = false;
    FLAG_found_rs_solution_ = false;

    fail_reasons_ = EnumFailReasons::not_fail; 

    counter_for_rs_search_ = 0 ;
    interval_for_rs_search_= 3 ;

    step_length_ = 0.1;
    turning_raius_ = 0.5;
    turning_angle_ = step_length_ / turning_raius_;

    obstacle_threshold_value_ = 65;

    grid_map_width_ = map_width_grid;   
    grid_map_height_ = map_height_grid; 

    // all_grids_[start_grid_].fine_pose = start_pose_;
    // all_grids_[start_grid_].parent = start_grid_;
    // all_grids_[start_grid_].state = EnumExploreState::OPEN;
    // all_grids_[start_grid_].steer_type = 1;

    open_grids_[start_grid_].fine_pose = start_pose_;
    open_grids_[start_grid_].parent = start_grid_;
    open_grids_[start_grid_].steer_type = 1;

    goal_tolerance.max_distance_error = 0.6;
    goal_tolerance.max_heading_error = 1.4;

    build_motion_model();

    std::cout << "Hybrid_astar_class::setup() DONE" << std::endl;
}




void Hybrid_astar_class::explore_one_node(std::array<double, 3> curr_pose, std::array<int, 3> curr_grid)
{
    // all_grids_[curr_grid].state = EnumExploreState::CLOSED;
    clos_grids_[curr_grid].fine_pose  = open_grids_[curr_grid].fine_pose;
    clos_grids_[curr_grid].gcost      = open_grids_[curr_grid].gcost;
    clos_grids_[curr_grid].fcost      = open_grids_[curr_grid].fcost;
    clos_grids_[curr_grid].parent     = open_grids_[curr_grid].parent;
    clos_grids_[curr_grid].steer_type = open_grids_[curr_grid].steer_type;
    open_grids_.erase(curr_grid);

    double curr_theta = curr_pose[2];
    double cos_theta = cos(curr_theta);
    double sin_theta = sin(curr_theta);

    bool usable_rs_found = false; 

    if( counter_for_rs_search_%interval_for_rs_search_ == 0){
        // check rs from this pose to goal
        RS_curve_finder_.setup(curr_pose, goal_pose_);
        RS_curve_finder_.search();
        // std::cout << "Found " << RS_curve_finder_.results_.size() << " possible rs paths " << std::endl;
        if( RS_curve_finder_.results_.size() >= 1){
            // int num_rs_path_to_check = std::min( int(RS_curve_finder_.results_.size()), 3 );
            int num_rs_path_to_check = int(RS_curve_finder_.results_.size())-1;
            int pci;
            for(pci=0; pci<=num_rs_path_to_check; pci++){
                auto *rs_path = &RS_curve_finder_.results_[pci];
                // std::cout << "check rs path #" << pci  << "  " << rs_path->path_word  << std::endl;
                
                if (rs_path->path_steps.size()==0){
                    std::cout << "it's an empty path" << std::endl;
                    continue;
                }
                // std::cout << "Not an empty path" << std::endl;
                bool this_rs_is_free_of_collision = true;
                for(int pointi = 1; pointi<=rs_path->path_steps.size(); pointi++){
                    // std::array<double,3> fine_pose = {rs_path->path_steps[pointi].x,
                    //                                     rs_path->path_steps[pointi].y,
                    //                                     rs_path->path_steps[pointi].theta};
                    // std::cout << "+-+-+-path_steps: " << rs_path->path_steps[pointi][0] << " " << rs_path->path_steps[pointi][1] << std::endl;
                    std::array<int, 3> point_grid = helper_convert_fine_pose_to_grid(rs_path->path_steps[pointi], fine_to_grid_ratio_, angle_resolution_);
                    int point_1d_index = twoD_to_oneD(point_grid[0], point_grid[1], grid_map_width_, grid_map_height_);
                    // std::cout << "+-+-+-grid: " << point_grid[0] << " " << point_grid[1] << " " << point_1d_index << std::endl;

                    if ( std::abs(rs_path->path_steps[pointi][0] - rs_path->path_steps[pointi-1][0]) + 
                         std::abs(rs_path->path_steps[pointi][1] - rs_path->path_steps[pointi-1][1])  > 0.4  ){
                            continue;
                         }

                    if( (*grid_map_)[ point_1d_index ] > obstacle_threshold_value_ ){
                        // std::cout << "rs path #" << pci << " collide" << std::endl;
                        usable_rs_found = false; 
                        this_rs_is_free_of_collision = false;
                        break;
                    }
                    // auto *p1 = &(rs_path->path_steps[pointi-1]);
                    // auto *p2 = &(rs_path->path_steps[pointi  ]);
                    // std::array<int, 3> p1_grid = helper_convert_fine_pose_to_grid();
                    // std::array<int, 3> p2_grid = helper_convert_fine_pose_to_grid();
                    // if(p1_grid == p2_grid){
                    //     // check only this grid for collision.
                    // }
                    // else{
                    //     // bresenham find line, and check each cell for collision. 
                    // }
                }
                if(this_rs_is_free_of_collision){
                    usable_rs_found = true;
                    break;
                }
            }
            if ( usable_rs_found ){
            rs_path_ = RS_curve_finder_.results_[pci].path_steps;
            FLAG_reach_goal_ = true;
            FLAG_found_rs_solution_ = true;
            start_grid_for_rs_curve_ = curr_grid; // helper_convert_fine_pose_to_grid(curr_pose, fine_to_grid_ratio_, angle_resolution_);
            close_goal_pose_ = curr_pose;
            return;
            }
        }
    }
    counter_for_rs_search_ ++;
    // std::cout << "RS section done." << std::endl;

    // cout << "explore_one_node:: " << all_grids_[curr_grid].fine_pose[0] << " " << all_grids_[curr_grid].fine_pose[1] << " "
    //  << all_grids_[curr_grid].fine_pose[2] << endl;

    double edge_cost = step_length_;
    int count = 0;
    for (auto mm : motion_model_)
    {
        double dx = cos_theta * mm[0] + sin_theta * mm[1];
        double dy = sin_theta * mm[0] + cos_theta * mm[1];
        double nb_x = curr_pose[0] + dx;
        double nb_y = curr_pose[1] + dy;

        // cout << "explore_one_node:: " << nb_x << " " << nb_y << " " << curr_theta + mm[2] << endl;

        int nb_x_grid = nb_x / fine_to_grid_ratio_;
        int nb_y_grid = nb_y / fine_to_grid_ratio_;

        if (0 <= nb_x_grid && nb_x_grid < grid_map_width_ && 0 <= nb_y_grid && nb_y_grid < grid_map_height_)
        {
            int nb_grid_index = twoD_to_oneD(nb_x_grid, nb_y_grid, grid_map_width_, grid_map_height_);

            if( (*grid_map_)[ nb_grid_index ] < obstacle_threshold_value_ )
            { // if not obstacle
                double nb_angle = mod_2pi(curr_theta + mm[2]); // angle change in this step

                const std::array<double, 3> nb_pose = {nb_x, nb_y, nb_angle};
                const std::array<int, 3> nb_grid = helper_convert_fine_pose_to_grid(nb_pose, fine_to_grid_ratio_, angle_resolution_);

                if ( open_grids_.count(nb_grid)==0 && clos_grids_.count(nb_grid)==0 ){
                    open_grids_[nb_grid].fine_pose = nb_pose;
                    open_grids_[nb_grid].parent = curr_grid;
                    open_grids_[nb_grid].steer_type = count;
                    double new_g_cost = clos_grids_[curr_grid].gcost  + edge_cost;
                    open_grids_[nb_grid].gcost = new_g_cost;
                    open_grids_[nb_grid].fcost = new_g_cost + compute_h_cost_Euclidean(nb_pose, goal_pose_);
                }
                else{
                    double old_cost;  bool to_update = false;
                    if ( open_grids_.count(nb_grid) > 0 ){
                        old_cost = open_grids_[nb_grid].fcost;
                        to_update = true;
                    }
                    else if ( clos_grids_.count(nb_grid) > 0 && clos_grids_[nb_grid].parent == curr_grid ){
                        old_cost = clos_grids_[nb_grid].fcost;
                        to_update = true;
                    }
                    if( to_update ){
                        double gcost = clos_grids_[curr_grid].gcost + edge_cost;
                        double fcost = gcost + compute_h_cost_Euclidean(nb_pose, goal_pose_);
                        if(fcost < old_cost){
                            open_grids_[nb_grid].fine_pose = nb_pose;
                            open_grids_[nb_grid].parent = curr_grid;
                            open_grids_[nb_grid].steer_type = count;
                            open_grids_[nb_grid].fcost = fcost;
                            open_grids_[nb_grid].gcost = gcost;
                            clos_grids_.erase(nb_grid);
                        }
                    }   
                }
                check_if_reach_goal(nb_pose, goal_pose_, curr_grid);
            }
        }
        count++;
    }
}


inline int Hybrid_astar_class::twoD_to_oneD(const int x, const int y, const int width, const int height)
{
    return y*width + x;
}




void Hybrid_astar_class::build_motion_model()
{
    double raius = turning_raius_;
    double angle = turning_angle_;
    // std::cout << "build_motion_model - turning_raius_: " << raius << std::endl;
    // std::cout << "build_motion_model - turning_angle_: " << angle << std::endl;

    double unit_dx = raius * sin(angle);
    double unit_dy = raius * (cos(angle) - 1);

    std::vector<double> temp = {unit_dx, -1 * unit_dy, angle, 0};
    motion_model_.push_back(temp);

    temp = {step_length_, 0, 0, 0};
    motion_model_.push_back(temp);

    temp = {unit_dx, unit_dy, double(M_PI * 2 - angle), 0};
    motion_model_.push_back(temp);

    temp = {-1 * unit_dx, unit_dy, angle, 1};
    motion_model_.push_back(temp);

    temp = {-1 * step_length_, 0, double(M_PI * 2 - angle), 1};
    motion_model_.push_back(temp);

    temp = {-1 * unit_dx, -1 * unit_dy, angle, 1};
    motion_model_.push_back(temp);
}


inline std::array<int, 3> Hybrid_astar_class::helper_convert_fine_pose_to_grid(const std::array<double, 3> fine_pose , double dist_ratio, double ang_ratio )
{
    std::array<int, 3> ct;
    ct[0] = int(fine_pose[0] / dist_ratio);
    ct[1] = int(fine_pose[1] / dist_ratio);
    ct[2] = int(fine_pose[2] / ang_ratio);
    return ct;
}




inline double Hybrid_astar_class::mod_2pi( double a)
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




double Hybrid_astar_class::compute_h_cost_Euclidean(const std::array<double, 3> n, const std::array<double, 3> g)
{
    double h = 0;

    double dx = n[0] - g[0];
    double dy = n[1] - g[1];

    h = sqrt(dx * dx + dy * dy) * 1;

    return (h);
}

double Hybrid_astar_class::compute_h_cost_Manhattan(const std::array<double, 3> n, const std::array<double, 3> g)
{
    double h = 0;

    double dx = abs(n[0] - g[0]);
    double dy = abs(n[1] - g[1]);

    h = (dx + dy) * 1;

    return (h);
}

double Hybrid_astar_class::compute_h_cost_Chebyshev(const std::array<double, 3> n, const std::array<double, 3> g)
{
    double h = 0;

    double dx = abs(n[0] - g[0]);
    double dy = abs(n[1] - g[1]);

    h = std::max(dx, dy) * 1;

    return (h);
}



std::vector<std::array<int, 3>> Hybrid_astar_class::find_min_cost_nodes()
{
    // std::cout << "find_min_cost_nodes" << std::endl;
    std::vector<std::array<int, 3>> out;
    double min_cost = std::numeric_limits<double>::max();
    std::array<int, 3> node;
    for ( auto n : open_grids_ ) 
    {
        node = n.first;
        double cost = n.second.fcost;
        if (cost < min_cost)
        {
            min_cost = cost;
            out.clear();
            out.push_back(node);
        }
        else if (cost == min_cost)
        {
            out.push_back(node);
        }
    }

    // std::cout << "Hybrid_astar_class::find_min_cost_nodes  DONE. " << "min_cost  "  << min_cost << "  size:" << out.size() << std::endl;

    return out;
}




void Hybrid_astar_class::explore_one_ite(std::vector<std::array<int, 3>> active_nodes)
{
    if (active_nodes.size()==0){
        FLAG_trapped_ = true;
        std::cout << "   !!!!!!!!!!!   TRAP   !!!!!!!!!!!!!! "<< std::endl;
        return;
    }

    for (std::vector<std::array<int, 3>>::iterator it = active_nodes.begin(); it != active_nodes.end(); ++it)
    {
        // std::cout << "explore node: " << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << std::endl;
        // auto fine_pose = all_grids_[*it].fine_pose;
        auto fine_pose = open_grids_[*it].fine_pose;
        explore_one_node(fine_pose, *it);
    }
}


bool Hybrid_astar_class::search()
{
    std::cout << "Hybrid_astar_class::search()  Looking for path " << std::endl;

    high_resolution_clock::time_point time1 = high_resolution_clock::now();

    int search_count = 0;
    while (!FLAG_reach_goal_)
    {
        // cout << "search_count " << search_count << endl;

        explore_one_ite( find_min_cost_nodes() );
        
        search_count ++;

        if ( FLAG_found_rs_solution_ ){
            std::cout << "search success, found_rs_solution, tried " << search_count << " times" << std::endl;
            break;
        }

        if( FLAG_trapped_ ){
            std::cout << "search trapped, failed, tried " << search_count << " times" << std::endl;
            return false;
        }

        high_resolution_clock::time_point time2 = high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time2-time1);

        if( int(duration.count()/1000.0) >= time_out_ms_ )
        {
            std::cout << "search timeout, failed, tried " << search_count << " times" << std::endl;
            return false;
        }


        // if(search_count < 3)
        // {
        //     for (auto n : all_grids_) 
        //     {
        //     cout << std::fixed << std::setprecision(3)  << "all grids: ["<< n.first[0]<< " " << n.first[1]<< " " << n.first[2]<< "]  g: " << n.second.gcost << " f: " << n.second.fcost << " ; ["
        //         << n.second.fine_pose[0] << " " << n.second.fine_pose[1] << " " << n.second.fine_pose[2] << "] "
        //         << n.second.state << " ["  << n.second.parent[0] << " "  << n.second.parent[1] << "] " << endl;
        //     }
        // }
    }

    std::cout << "Found the goal. Tried " << search_count << " times " << std::endl;
    
    return true;
}



void Hybrid_astar_class::get_path( std::deque< array<double, 3> >& path)
{
    path.clear();

    std::array<int, 3> get_path_curr_node;

    if (FLAG_found_rs_solution_){
        get_path_curr_node = start_grid_for_rs_curve_;
    }
    else{
        get_path_curr_node = helper_convert_fine_pose_to_grid(close_goal_pose_, fine_to_grid_ratio_, angle_resolution_);
    }

    // array<double, 3>  point;
    while (get_path_curr_node != start_grid_)
    {
        // if (get_path_curr_node[0] != 0){
        //     std::cout << "goal_grid_  : " << goal_grid_[0] << " " << goal_grid_[1] << " " << goal_grid_[2] << std::endl;
        //     std::cout << "start_grid_ : " << start_grid_[0] << " " << start_grid_[1] << " " << start_grid_[2] << std::endl;
        //     std::cout << "this        : " << get_path_curr_node[0] << " " << get_path_curr_node[1] << " " << get_path_curr_node[2] << std::endl;
        // }

        // path.push_front( all_grids_[get_path_curr_node].fine_pose );
        path.push_front( clos_grids_[get_path_curr_node].fine_pose );
        
        get_path_curr_node = clos_grids_[get_path_curr_node].parent;
    }

    if( FLAG_found_rs_solution_ ){
        for( auto pt : rs_path_ ){
            path.push_back( pt  );
        }
    }

    // std::cout << "Found path: " << std::endl;
    // for ( auto pht : path){
    //     std::cout << pht[0] << " " << pht[1] << " " << pht[2] << std::endl;
    // }
}


void Hybrid_astar_class::check_if_reach_goal(array<double, 3> node_pose,array<double, 3> in_goal_pose, array<int, 3> in_curr_grid){
    bool position_close = compute_h_cost_Euclidean(node_pose, in_goal_pose) < goal_tolerance.max_distance_error;
    bool yaw_angl_close = abs(node_pose[2] - in_goal_pose[2]) < goal_tolerance.max_heading_error ;
    if ( position_close && yaw_angl_close )
    {
        FLAG_reach_goal_ = true;
        close_goal_angle_ = node_pose[2];
        close_goal_pose_ = node_pose;
        clos_grids_[goal_grid_].parent = in_curr_grid;
        // grid_parent[goal_grid] = in_curr_grid;
        clos_grids_[goal_grid_].fine_pose = node_pose;
        // grid_fine_pose[goal_grid] = node_pose;
    }
}



#endif
