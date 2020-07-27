#ifndef HA_NODE_H
#define HA_NODE_H

#include "hybrid_astar/map.h"


namespace ha
{

//******************* global variables **********************//
// std::shared_ptr<map_t> global_cost_map;
map_t* global_cost_map;
map_t* global_obs_map;

double CAR_LENGTH = 34;
double CAR_WIDTH = 20;
double DELTA_THETA = 15; // degree // wheel max turnning angle
double DELTA_FORWARD = 40; // (m) // distance to move forward

int THETA = 72; // degree // this is the zoomed out version of 360 degrees 
int THETA_RES = 5; // degree // Theta*Res = 360
int GX = 80;
int GY = 80;
int GRID_RES;

// cost map params
double DX = 400; // Astar cost_map resolution
double DY = 400;

// for collision check
double OBS_DIST_MIN = 8; //(m) // minimum dist to nearist obstacle
double DELTA_CHECK = 0.2; //(m) // delta for distance between points 
//***********************************************************//

class Ha_node
{

public:

    ros::NodeHandle nh;   

    ros::Publisher particlecloud_pub_;
    ros::Publisher global_map_pub_;

    int loop_freq = 10;

    std::string path;
    std::string obs_path;

    Map ha_map;

    Pose_vector start_pose;
    Pose_vector end_pose;
    std::vector<Pose_vector> final_path;
    std::vector<Pose_vector> current_set;

    nav_msgs::OccupancyGrid pub_obs_map;

    bool init_flag = 0;
    Pose_vector* head;
    

    Ha_node(); 

    void initialize(void);  

    void process(void);

    void hybridAstar(void);

    void reconstruct_path(Pose_vector current);

    void particlecloud_publisher(ros::Publisher particlecloud_pub_, std::vector<Pose_vector> sample_set);

    
    // void get_node_params(void);

    

private:
    /* data */
    bool path_valid = 0;

    std::vector<std::vector<std::vector<int>>> vis;
    // int*** vis;

    std::vector<std::vector<std::vector<Pose_vector>>> previous_set;
};

// calculate cost for hybridAstar
double holonomic_with_obs(Pose_vector p)
{
	int index_temp = p.dx + p.dy * global_cost_map->size_x;
	return global_cost_map->cells[index_temp].cost;
}

// for hybridAstar queue
struct cmp1{
    bool operator() ( Pose_vector a, Pose_vector b )
    {         
        return a.cost3d+holonomic_with_obs(a) > b.cost3d+holonomic_with_obs(b); 
    }
};


}

#endif