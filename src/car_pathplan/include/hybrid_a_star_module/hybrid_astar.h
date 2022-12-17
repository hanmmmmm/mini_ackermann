
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

// #include "../local_planner/path_data.h"

using std::array;
using std::cout;
using std::endl;
using std::string;
using std::vector;

using std::chrono::high_resolution_clock;


class Hybrid_astar_class  // :public pathPoint
{
private:

    vector<int8_t> *grid_map_;

    int grid_map_width_, grid_map_height_;

    float fine_to_grid_ratio_ = 0.0;
    const float angle_resolution_ = M_PI/18.0; // radian


    int time_out_ms_;

    vector< vector<float> > motion_model_;

    array<int, 3> start_grid_;
    array<float, 3> start_pose_;

    array<int, 3> goal_grid_;
    array<float, 3> goal_pose_;

    array<float, 3> close_goal_pose_;

    float start_angle_, goal_angle_, close_goal_angle_; 

    struct gridInfo
    {
        float gcost = 0;
        float fcost = 0;
        int state = 0;       // 0:new   1:open   2:closed
        int steer_type = 0;  // 0-5, total 6 types
        // float fine_pose_x, fine_pose_y, fine_pose_yaw;
        array<float, 3> fine_pose;
        array<int,3> parent;
    };
    
    std::unordered_map<array<int, 3>, gridInfo , ArrayHasher3> all_grids_;

    struct goalTolerance
    {
       float max_distance_error;
       float max_heading_error;
    };

    goalTolerance goal_tolerance;
   


    // int fine_map_width_, fine_map_height_, grid_map_width_, grid_map_height_;

    float step_length_ ;
    float turning_raius_  ;
    float turning_angle_ ;

    bool FLAG_reach_goal_ = false;


    std::deque< array<float, 3> > path_;

    void build_motion_model();

    void print_init_info();

    float mod_2pi(float angle);


    int improve_hcost( array<float, 3> node_pose, int hcost, int scan_range, bool same_steer);
    void check_if_reach_goal( array<float, 3> node_pose, array<float, 3> in_goal_pose, array<int, 3> in_curr_grid);

    int twoD_to_oneD(const int x, const int y, const int width, const int height);

    std::array<int, 3> helper_convert_fine_pose_to_grid(const std::array<float, 3> fine_pose , float dist_ratio, float ang_ratio );

    float compute_h_cost_Euclidean(const std::array<float, 3> n, const std::array<float, 3> g);
    float compute_h_cost_Manhattan(const std::array<float, 3> n, const std::array<float, 3> g);
    float compute_h_cost_Chebyshev(const std::array<float, 3> n, const std::array<float, 3> g);


public:
    Hybrid_astar_class();

    void setup(const int timeout_ms, 
                const array<float,3> startpose, 
                const array<float,3> goalpose ,
                const int map_width_grid, 
                const int map_height_grid, 
                vector<int8_t>& map,
                const float grid_resolution);

    ~Hybrid_astar_class();

    bool FLAG_update_map_for_view = true;

    void explore_one_node(array<float, 3> curr_pose, array<int, 3> curr_grid);
    void explore_one_ite(vector< array<int, 3>> active_nodes);
    vector< array<int,3>> find_min_cost_nodes();
    bool search();
    std::deque< array<float, 3> >  get_path();
    void print_path();


};



Hybrid_astar_class::Hybrid_astar_class() 
{
}

Hybrid_astar_class::~Hybrid_astar_class() 
{
}


void Hybrid_astar_class::setup(
                                const int timeout_ms,
                                const array<float,3> startpose, 
                                const array<float,3> goalpose,
                                const int map_width_grid, 
                                const int map_height_grid, 
                                vector<int8_t>& map,
                                const float grid_resolution 
                                )
{
    time_out_ms_ = timeout_ms;

    grid_map_ = &map;

    start_pose_ = startpose;
    goal_pose_  = goalpose ;

    fine_to_grid_ratio_ = grid_resolution ;

    start_grid_ = helper_convert_fine_pose_to_grid(start_pose_, fine_to_grid_ratio_, angle_resolution_);
    goal_grid_  = helper_convert_fine_pose_to_grid(goal_pose_,  fine_to_grid_ratio_, angle_resolution_);

    std::cout << "start_grid_ " << std::endl;
    std::cout << start_grid_[0] << " " << start_grid_[1] << " " << start_grid_[2] << std::endl;
    std::cout << start_pose_[0] << " " << start_pose_[1] << " " << start_pose_[2] << std::endl;

    std::cout << "goal_grid_ " << std::endl;
    std::cout << goal_grid_[0] << " " << goal_grid_[1] << " " << goal_grid_[2] << std::endl;
    std::cout << goal_pose_[0] << " " << goal_pose_[1] << " " << goal_pose_[2] << std::endl;

    path_.clear();
    all_grids_.clear();

    FLAG_reach_goal_ = false;

    step_length_ = 0.2;
    turning_raius_ = 0.5;
    turning_angle_ = step_length_ / turning_raius_;

    grid_map_width_ = map_width_grid;   
    grid_map_height_ = map_height_grid; 

    all_grids_[start_grid_].fine_pose = start_pose_;
    all_grids_[start_grid_].parent = start_grid_;
    all_grids_[start_grid_].state = 1;
    all_grids_[start_grid_].steer_type = 1;

    goal_tolerance.max_distance_error = 0.6;
    goal_tolerance.max_heading_error = 1.4;

    build_motion_model();

    // print_init_info();
}




void Hybrid_astar_class::explore_one_node(std::array<float, 3> curr_pose, std::array<int, 3> curr_grid)
{
    all_grids_[curr_grid].state = 2;
    float curr_theta = curr_pose[2];
    float cos_theta = cos(curr_theta);
    float sin_theta = sin(curr_theta);

    // cout << "explore_one_node:: " << all_grids_[curr_grid].fine_pose[0] << " " << all_grids_[curr_grid].fine_pose[1] << " "
    //  << all_grids_[curr_grid].fine_pose[2] << endl;

    int count = 0;
    for (auto mm : motion_model_)
    {
        float dx = cos_theta * mm[0] + sin_theta * mm[1];
        float dy = sin_theta * mm[0] + cos_theta * mm[1];
        float nb_x = curr_pose[0] + dx;
        float nb_y = curr_pose[1] + dy;

        // cout << "explore_one_node:: " << nb_x << " " << nb_y << " " << curr_theta + mm[2] << endl;

        int nb_x_grid = nb_x / fine_to_grid_ratio_;
        int nb_y_grid = nb_y / fine_to_grid_ratio_;

        if (0 <= nb_x_grid && nb_x_grid < grid_map_width_ && 0 <= nb_y_grid && nb_y_grid < grid_map_height_)
        {
            
            int nb_grid_index = twoD_to_oneD(nb_x_grid, nb_y_grid, grid_map_width_, grid_map_height_);
            int obs_prob = int((*grid_map_)[ nb_grid_index ]);
            if( obs_prob < 90 )
            { // if not obstacle

                float edge_cost = step_length_;
                int is_reversing = int(mm[3]);
                if (is_reversing)
                    {edge_cost *= 1.05;}

                int nb_steer = count;
                // int parent_steer = grid_status[curr_grid][3];
                int parent_steer = all_grids_[curr_grid].steer_type;

                float nb_a = curr_theta + mm[2]; // angle change in this step
                nb_a = mod_2pi(nb_a);

                const std::array<float, 3> nb_pose = {nb_x, nb_y, nb_a};
                const std::array<int, 3> nb_grid = helper_convert_fine_pose_to_grid(nb_pose, fine_to_grid_ratio_, angle_resolution_);

                // if (grid_parent.count(nb_grid) == 0)
                if (all_grids_.count(nb_grid) == 0)
                {
                    all_grids_[nb_grid].fine_pose = nb_pose;
                    all_grids_[nb_grid].parent = curr_grid;
                    all_grids_[nb_grid].state = 1;
                    all_grids_[nb_grid].steer_type = count;
                    float gcost = all_grids_[curr_grid].gcost  + edge_cost;
                    all_grids_[nb_grid].gcost = gcost;

                    float hcost = compute_h_cost_Euclidean(nb_pose, goal_pose_);

                    // hcost = improve_hcost(nb_pose, hcost, obstacle_scan_range, nb_steer==parent_steer);

                    all_grids_[nb_grid].fcost = gcost + hcost ;
                }
                else{
                    if( all_grids_[nb_grid].state==1  ||  (all_grids_[nb_grid].state==2 && all_grids_[nb_grid].parent==curr_grid ) ){
                        float old_cost = all_grids_[nb_grid].fcost;
                        float gcost = all_grids_[curr_grid].gcost + edge_cost;
                        all_grids_[nb_grid].gcost = gcost;

                        float hcost = compute_h_cost_Euclidean(nb_pose, goal_pose_);
                        // hcost = improve_hcost(nb_pose, hcost, obstacle_scan_range, nb_steer==parent_steer);
                        float fcost = gcost + hcost;
                        
                        if(fcost < old_cost){
                            all_grids_[nb_grid].fine_pose = nb_pose;
                            all_grids_[nb_grid].parent = curr_grid;
                            all_grids_[nb_grid].state = 1;
                            all_grids_[nb_grid].steer_type = count;
                            all_grids_[nb_grid].fcost = fcost;
                            all_grids_[nb_grid].gcost = gcost;
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
    float raius = turning_raius_;
    float angle = turning_angle_;
    // std::cout << "build_motion_model - turning_raius_: " << raius << std::endl;
    // std::cout << "build_motion_model - turning_angle_: " << angle << std::endl;

    float unit_dx = raius * sin(angle);
    float unit_dy = raius * (cos(angle) - 1);

    std::vector<float> temp = {unit_dx, -1 * unit_dy, angle, 0};
    motion_model_.push_back(temp);

    temp = {step_length_, 0, 0, 0};
    motion_model_.push_back(temp);

    temp = {unit_dx, unit_dy, float(M_PI * 2 - angle), 0};
    motion_model_.push_back(temp);

    temp = {-1 * unit_dx, unit_dy, angle, 1};
    motion_model_.push_back(temp);

    temp = {-1 * step_length_, 0, float(M_PI * 2 - angle), 1};
    motion_model_.push_back(temp);

    temp = {-1 * unit_dx, -1 * unit_dy, angle, 1};
    motion_model_.push_back(temp);
}


std::array<int, 3> Hybrid_astar_class::helper_convert_fine_pose_to_grid(const std::array<float, 3> fine_pose , float dist_ratio, float ang_ratio )
{
    std::array<int, 3> ct;
    ct[0] = int(fine_pose[0] / dist_ratio);
    ct[1] = int(fine_pose[1] / dist_ratio);
    ct[2] = int(fine_pose[2] / ang_ratio);
    return ct;
}




float Hybrid_astar_class::mod_2pi( float a)
{
    float angle = a;
    while (angle > 2*M_PI){
        angle -= 2*M_PI;
    }
    while (angle < 0){
        angle += 2*M_PI;
    }
    return angle;
}




float Hybrid_astar_class::compute_h_cost_Euclidean(const std::array<float, 3> n, const std::array<float, 3> g)
{
    float h = 0;

    float dx = n[0] - g[0];
    float dy = n[1] - g[1];

    h = sqrt(dx * dx + dy * dy) * 1;

    return (h);
}

float Hybrid_astar_class::compute_h_cost_Manhattan(const std::array<float, 3> n, const std::array<float, 3> g)
{
    float h = 0;

    float dx = abs(n[0] - g[0]);
    float dy = abs(n[1] - g[1]);

    h = (dx + dy) * 1;

    return (h);
}

float Hybrid_astar_class::compute_h_cost_Chebyshev(const std::array<float, 3> n, const std::array<float, 3> g)
{
    float h = 0;

    float dx = abs(n[0] - g[0]);
    float dy = abs(n[1] - g[1]);

    h = std::max(dx, dy) * 1;

    return (h);
}



std::vector<std::array<int, 3>> Hybrid_astar_class::find_min_cost_nodes()
{
    // std::cout << "find_min_cost_nodes" << std::endl;
    std::vector<std::array<int, 3>> out;
    float min_cost = std::numeric_limits<float>::max();
    
    // for (auto n : grid_status) // if a node exist in node_parent
    for (auto n : all_grids_) 
    {
        std::array<int, 3> node = n.first;
        if (n.second.state == 1)  // if state == open
        {
            float cost = n.second.fcost;
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

    }

    // cout << "min_cost  " << min_cost << endl;

    return out;
}




void Hybrid_astar_class::explore_one_ite(std::vector<std::array<int, 3>> active_nodes)
{
    for (std::vector<std::array<int, 3>>::iterator it = active_nodes.begin(); it != active_nodes.end(); ++it)
    {
        // auto fine_pose = grid_fine_pose[*it];
        auto fine_pose = all_grids_[*it].fine_pose;
        explore_one_node(fine_pose, *it);
    }


    
}





bool Hybrid_astar_class::search()
{
    std::cout << "Looking for path " << std::endl;

    high_resolution_clock::time_point time1 = high_resolution_clock::now();

    int search_count = 0;
    while (!FLAG_reach_goal_)
    {
        explore_one_ite( find_min_cost_nodes() );
        // cout << "search_count " << search_count << endl;
        search_count ++;

        high_resolution_clock::time_point time2 = high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time2-time1);

        if( int(duration.count()/1000.0) >= time_out_ms_ )
        {
            cout << "search failed, tried " << search_count << "times ,timeout" << std::endl;
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

    std::cout << "Found the goal" << std::endl;
    
    return true;
}




std::deque< array<float, 3> >  Hybrid_astar_class::get_path()
{
    std::array<int, 3> get_path_curr_node = helper_convert_fine_pose_to_grid(close_goal_pose_, fine_to_grid_ratio_, angle_resolution_);
    // std::cout << "get_path" << std::endl;
    
    while (get_path_curr_node != start_grid_)
    {
        // cout << "Hybrid_astar_class::get_path: " << all_grids_[get_path_curr_node].fine_pose[0] << " " << all_grids_[get_path_curr_node].fine_pose[1] << " " << all_grids_[get_path_curr_node].fine_pose[2] << endl;
        array<float, 3>  point;
        point[2] = all_grids_[get_path_curr_node].fine_pose[2];
        point[0] = all_grids_[get_path_curr_node].fine_pose[0];
        point[1] = all_grids_[get_path_curr_node].fine_pose[1];
        path_.push_front( point );
        
        get_path_curr_node = all_grids_[get_path_curr_node].parent;
    }

    return path_;
}


void Hybrid_astar_class::check_if_reach_goal(array<float, 3> node_pose,array<float, 3> in_goal_pose, array<int, 3> in_curr_grid){
    if (compute_h_cost_Euclidean(node_pose, in_goal_pose) < goal_tolerance.max_distance_error)
    {
        if (abs(node_pose[2] - in_goal_pose[2]) < goal_tolerance.max_heading_error  )
        {
            FLAG_reach_goal_ = true;
            close_goal_angle_ = node_pose[2];
            close_goal_pose_ = node_pose;
            all_grids_[goal_grid_].parent = in_curr_grid;
            // grid_parent[goal_grid] = in_curr_grid;
            all_grids_[goal_grid_].fine_pose = node_pose;
            // grid_fine_pose[goal_grid] = node_pose;
            all_grids_[goal_grid_].state = 2;
            // grid_status[goal_grid][2] = 2;
        }
    }
}



#endif
