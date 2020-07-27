#ifndef POSE_H
#define POSE_H

#include "hybrid_astar/hastar.h"

namespace ha
{

class Pose_vector
{
public:

    double v[3] = {0};
    double gscore = 0; // cost for dijkstra
    double fscore = 0; // hueristic cost for Astar
    double cost3d = 0; // cost for hybridAstar
    int dx = 0; // coord for Astar
    int dy = 0; // coord for Astar
    int gx = 0; // coord for hybridAstar
    int gy = 0; // coord for hybridAstar
    int gtheta = 0; // heading for HybridAstar

    int previous_num = -1; // previous node num in current set

    bool start_flag = 0;

    std::vector<Pose_vector> getNextPose(void);

    void After(Pose_vector *p);
    
    Pose_vector* previous = NULL;
    Pose_vector* next = NULL;

    
private:
    /* data */
};

Pose_vector* CreateNode(void);
void Insert(Pose_vector *head, Pose_vector *pprevious);

// for Astar queue
struct cmp0{
    bool operator() ( Pose_vector a, Pose_vector b )
    {         
        // return a.fscore > b.fscore; 
        return a.gscore > b.gscore; 
    }
};

}

#endif
