#ifndef MAP_H
#define MAP_H

#include "hybrid_astar/hastar.h"
#include "hybrid_astar/pose.h"

namespace ha
{

typedef struct
{
    // Occupancy state (0 = free, +1 = occ)
    int occ_state = 0;

    // Distance to the nearest occupied cell (-1 = inside, +1 = edge, >1 = free space)
    double occ_dist = 0;

    // Cost for Astar
    float cost = 0;

} map_cell_t;

// occupancy grid map
// Description for a map
typedef struct
{
    // Map origin; the map is a viewport onto a conceptual larger map.
    double origin_x, origin_y;
        
    // Map scale (m/cell)
    double scale;

    // Map dimensions (number of cells)
    int size_x, size_y;
        
    // The map data, stored as a grid
    map_cell_t *cells;

    // Max distance at which we care about obstacles, for constructing
    // likelihood field
    double max_occ_dist;
    
} map_t;


class Map
{

public:

    map_t* global_map; // global obs occupency map // square map

    map_t* cost_map; // djkstra cost_map // square map

    int max_dist_index; // for global map

    bool obs_init_flag = false;

    nav_msgs::OccupancyGridPtr pub_obs_map_ptr;


    //---------------------------------------------------------------

    map_t* map_alloc(void);

    Map();

    ~Map();

    void mapRequest();
    
    void mapReceived(const nav_msgs::OccupancyGrid& map_msg);

    void mapFromPic(std::string obs_path);

    // check if the obs cell is boundary or not
    bool isBoundary(int i, int j);

    // update obs_map, cells away from obs get bigger obs_dist, cells on the boundary get 1, cells inside obs gets -1
    void obsUpdate();

    // initialize cost map
    void cost_map_init();

    // update cost map
    void runAstar(Pose_vector start_pose, Pose_vector end_pose);

    bool checkCollision(Pose_vector current);

    double holonomic_with_obs(Pose_vector p);


    void free_map(int type); // 1 for global_map, 2 for cost_map, other for all


private:
    /* data */

    float max_cost2d = 0;
};




}

#endif