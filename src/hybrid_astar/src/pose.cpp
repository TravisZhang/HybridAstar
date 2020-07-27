#include "hybrid_astar/pose.h"

namespace ha
{

std::vector<Pose_vector> Pose_vector::getNextPose(void)
{
    std::vector<Pose_vector> nextset;
    Pose_vector pose_temp;

    double d_theta,beta,x,y,r,heading,h_temp;
    heading = v[2]*M_PI/180;

    // ROS_INFO("x_c: %.3f, y_c: %.3f, h_c: %.3f\n",v[0],v[1],v[2]);

    for(d_theta = -DELTA_THETA;d_theta <= DELTA_THETA+0.001;d_theta += DELTA_THETA)
    {
        r = CAR_LENGTH / tan(d_theta*M_PI/180);
        // beta = DELTA_FORWARD / r;
        beta = DELTA_FORWARD * tan(d_theta*M_PI/180) / CAR_LENGTH;
        // ROS_INFO("beta: %.3f, r: %.3f",beta,r);

        if(beta<0.001&&beta>-0.001)
        {
            x = v[0] + DELTA_FORWARD * cos(heading);
            y = v[1] + DELTA_FORWARD * sin(heading);
            h_temp = heading;
            // ROS_INFO("heading_temp0: %.3f",h_temp);
        }
        else
        {
            x = v[0] + r * sin(beta + heading) - r * sin(heading);
            y = v[1] - r * cos(beta + heading) + r * cos(heading);
            // x = v[0] + DELTA_FORWARD * cos(beta + heading);
            // y = v[1] + DELTA_FORWARD * sin(beta + heading);
            h_temp = heading + beta;
            // ROS_INFO("heading_temp1: %.3f",h_temp);
            if(h_temp > 0)
            {
                h_temp = fmod(h_temp, 2*M_PI);
            }
            else
            {
                while(h_temp < 0)
                    h_temp += 2*M_PI;
            }
        }
        pose_temp.v[0] = x;
        pose_temp.v[1] = y;
        pose_temp.v[2] = (h_temp)*180/M_PI;

        // ROS_INFO("heading_temp: %.3f",pose_temp.v[2]);
        // ROS_INFO("x_temp: %.3f, y_temp: %.3f, h_temp: %.3f\n",pose_temp.v[0],pose_temp.v[1],pose_temp.v[2]);

        nextset.push_back(pose_temp);
    }

    // ROS_INFO("beta: %.3f\n",beta);

    return nextset;
}

Pose_vector* CreateNode(void)
{
    Pose_vector *newNode = (Pose_vector*)malloc(sizeof(Pose_vector));
    if (newNode == NULL) 
    {
        printf("out of memory!\n");
        return NULL;
    } else
    {
        newNode->previous = NULL;
        return newNode;
    }
}

void Insert(Pose_vector *head, Pose_vector *pprevious)
{
    Pose_vector *newNode = pprevious;
    newNode->previous = head->previous;
    head->previous = newNode;
} 

void Pose_vector::After(Pose_vector *p)
{
    previous = (Pose_vector*)malloc(sizeof(Pose_vector));
    previous = p;
} 

}

