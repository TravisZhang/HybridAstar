//
//  motion_plan_pub
//  decision
//
//  Created by Chaoyu Zhang on 05/07/2018.
//  Copyright (c) 2015 Chaoyu Zhang. All rights reserved.
//

#include "motion_plan/motion_plan.h"

#define SQRT2 1.4142135623
#define MAX_OCC_DIST 0.2
#define MAX_STEERING_ANGLE 30

//yaw: (-pi~pi,x->0)
bool method_switch = 0; //0 for nodes, 1 for sample points

std::vector< pose_vector > sample_set;
std::vector< int > index_set;
std::vector< std::vector<double> > angle_ref_set;
map_t* global_map;

bool isP(double i,double j)
{
  return i==j;
}

bool compareP(double i,double j)
{
  return i>j;
}

// Create a new map
map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*) malloc(sizeof(map_t));

  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;
  
  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;
  
  // Allocate storage for main map
  map->cells = (map_cell_t*) NULL;
  
  return map;
}

void update_occ_dist(map_t* map){

	int map_size = map->size_x * map->size_y;
	int i,j,i0,j0,dist,index_x,index_y,index,index_m;
	double current_dist;

	for(i = 0;i < map_size;i++){		
		if(map->cells[i].occ_state != 1)
			map->cells[i].occ_dist = map->max_occ_dist;		
		else if(map->cells[i].occ_state == 1)
      		map->cells[i].occ_dist = 0;
	}

	dist = (int) ceil(map->max_occ_dist / map->scale);

	for(j = 0;j < map->size_y;j++){
		for(i = 0;i < map->size_x;i++){
			index_m = i + j * map->size_x;
			if((map->cells[index_m].occ_state != 1)){
				// ROS_INFO("continue 1");
				continue;
			}


			for(j0 = -dist;j0 <= dist;j0++){
				for(i0 = -dist;i0 <= dist;i0++){
					index_x = i+i0;
					index_y = j+j0;
					if((index_x<0)||(index_x>=map->size_x)||(index_y<0)||(index_y>=map->size_y)){
						// ROS_INFO("continue 2");
						continue;
					}

					
					index = index_x + index_y * map->size_x;
					current_dist = sqrt(j0*j0+i0*i0)*map->scale;
					// ROS_INFO("map cell occ_dist judge,  idx: %d, current: %.3f before: %.3f",index,current_dist,map->cells[index].occ_dist);
					if(current_dist < map->cells[index].occ_dist)
						map->cells[index].occ_dist = current_dist;
						// ROS_INFO("map cell occ_dist,  idx: %d, value: %.3f \n",index,current_dist);
				}
			}
			// ROS_INFO("map cell occ_dist calculating,  idx: %d \n",index_m);
		}
	}

}

void initialSampleGenerator(std::vector< pose_vector >& sample_set, map_t* map){
	int i,j;
	int map_size = map->size_x * map->size_y;
	int index;
	pose_vector sample_point;

	for(j = 0;j < map->size_y;j++)
	{
		for(i = 0;i < map->size_x;i++)
		{
			index = i + j * map->size_x;
			if(map->cells[index].occ_state != -1)
			{
				continue;
			}

			sample_point.v[0] = map->origin_x + i * map->scale + 0.5 * map->scale;
			sample_point.v[1] = map->origin_y + j * map->scale + 0.5 * map->scale;
			sample_point.v[2] = 0;
			sample_set.push_back(sample_point);
			index_set.push_back(index);

		}
	}
 
}

void mapReceived(const nav_msgs::OccupancyGrid& map_msg)
{
	ROS_INFO("Received a %d X %d map @ %.3f m/pix, Origin x: %.3f, y: %.3f \n",
           map_msg.info.width,
           map_msg.info.height,
           map_msg.info.resolution,
		   map_msg.info.origin.position.x,
		   map_msg.info.origin.position.y);

	map_t* map = map_alloc();
  	ROS_ASSERT(map);

  	map->size_x = map_msg.info.width;
  	map->size_y = map_msg.info.height;
  	map->scale = map_msg.info.resolution;
  	map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale; //prevent negative coords
  	map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale; //prevent negative coords
  	// Convert to player format
	//  The map data, in row-major order, starting with (0,0).  Occupancy
	//  probabilities are in the range [0,100].  Free is -1.
  	map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  	ROS_ASSERT(map->cells);
  	for(int i=0;i<map->size_x * map->size_y;i++)
  	{
    	if(map_msg.data[i] == 0)
      		map->cells[i].occ_state = -1;
    	else if(map_msg.data[i] == 100)
      		map->cells[i].occ_state = +1;
    	else
      		map->cells[i].occ_state = 0;
  	}
	global_map = map;
	ROS_INFO("Calculate occ_dist");
	//update max_occ_dist
	global_map->max_occ_dist = MAX_OCC_DIST;
	update_occ_dist(global_map);
	ROS_INFO("Calculate occ_dist done");

	sample_set.clear();
	sample_set.reserve(map->size_x * map->size_y);
	index_set.clear();
	index_set.reserve(map->size_x * map->size_y);
	index_set.resize(map->size_x * map->size_y,0);
	ROS_INFO("sample_set reset\n");
	initialSampleGenerator(sample_set,map);
	ROS_INFO("sample_set generated\n");
}

void mapRequest(){

  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  mapReceived( resp.map );
}

double distance_calculator(pose_vector start_node, pose_vector end_node){
	double inf=1.0/0.0;
	pose_vector temp_pose;
	int index_x, index_y, current_index;
	double delta = global_map->scale/2;
	double delta_x = end_node.v[0] - start_node.v[0];
	double delta_y = end_node.v[1] - start_node.v[1]; 
	double dist = sqrt(delta_x * delta_x + delta_y * delta_y);
	int max_num = floor(dist / delta);
	double angle =  std::atan2(delta_y, delta_x) / M_PI * 180;
	delta_x = delta * cos(angle);
	delta_y = delta * sin(angle);

	for(int i = 0;i < max_num;++i)
	{
		temp_pose.v[0] = start_node.v[0] + delta_x * i;
		temp_pose.v[1] = start_node.v[1] + delta_y * i;
		index_x = floor((temp_pose.v[0] - global_map->origin_x) / global_map->scale + 0.5);
		index_y = floor((temp_pose.v[1] - global_map->origin_x) / global_map->scale + 0.5);
		current_index = index_x + index_y * global_map->size_x;
		if(global_map->cells[current_index].occ_state != -1)
		{
			dist = inf;
			break;
		}
	}

	return dist;

}

void particlecloud_publisher(ros::Publisher particlecloud_pub_, std::vector<pose_vector> sample_set){
	// ros::Rate r(1/LOOP_DURATION);
	ROS_INFO("Pointcloud publish Begin >>>>>>>>>>>>> \n");
	geometry_msgs::PoseArray cloud_msg;
	std::vector< pose_vector > pub_set;
	// if(sample_cloud_pub_flag == 0) pub_set = sample_set;
	// else pub_set = resample_set;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
    cloud_msg.poses.resize(sample_set.size());
    for(int i=0;i<sample_set.size();i++)
    {
    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(sample_set[i].v[2]),
                                tf::Vector3(sample_set[i].v[0],
                                        sample_set[i].v[1], 0)),
                    cloud_msg.poses[i]);
    }
    particlecloud_pub_.publish(cloud_msg);
	ROS_INFO("Pointcloud publish Done >>>>>>>>>>>>> \n");
	// r.sleep();
}

void motion_plan_processor(pose_vector start_pose, pose_vector end_pose, ros::Publisher particlecloud_pub_){
	std::vector<int> unsettled_set;
	std::vector<int> settled_set;
	std::vector<bool> visited_status_set;
	std::vector<double> value_set;
	std::vector<int> last_node_set;
	std::vector<int> angle_set;

	int i,j,k;
	int index_temp,index_temp1;
	double value_temp,value_temp1,angle_diff;
	
	double inf=1.0/0.0;
	int totalNumber = global_map->size_x * global_map->size_y;
	last_node_set.reserve(totalNumber);
	last_node_set.resize(totalNumber);
	value_set.reserve(totalNumber);
	value_set.resize(totalNumber,inf); //set all nodes' value to inf
	visited_status_set.reserve(totalNumber);
	visited_status_set.resize(totalNumber,0);
	angle_set.reserve(totalNumber);
	angle_set.resize(totalNumber,0);

	ROS_INFO("inf is %lf\n",inf);

	int index_x = floor((start_pose.v[0] - global_map->origin_x) / global_map->scale + 0.5);
	int index_y = floor((start_pose.v[1] - global_map->origin_x) / global_map->scale + 0.5);
	int start_index = index_x + index_y * global_map->size_x;
	int current_index = start_index; //set start node as current node
	ROS_INFO("index_x : %d, index_y : %d, current_index: %d\n",index_x,index_y,current_index);
	value_set[current_index] = 0; //set current node's value to 0;
	angle_set[current_index] = start_pose.v[2];

	unsettled_set.push_back(current_index); //add current node to unsettled set
	visited_status_set[current_index] = 1; //set current node's visited status to 1

	// ROS_INFO("isP = %d",isP(1,1));

	// ROS_INFO("compareP = %d",compareP(inf,1));

	ROS_WARN("ENTERING WHILE LOOP >>>>>>>>\n");

	// main while loop
	while(unsettled_set.size() != 0){
		for(j = index_y-1;j <= index_y+1; ++j){
			for(i = index_x-1;i <= index_x+1;++i){

				// ROS_INFO("i: %d, j: %d\n",i,j);
				index_temp = i + j * global_map->size_x;
				angle_diff = angle_set[current_index] - angle_ref_set[i - index_x + 1][j - index_y + 1];

				//the neighbor node has to be free and can not be itself and must be unvisited and must has a small angle diff
				if((global_map->cells[index_temp].occ_state != -1)
					||(visited_status_set[index_temp] == 1)
					||(global_map->cells[index_temp].occ_dist < MAX_OCC_DIST)
					||(angle_diff > MAX_STEERING_ANGLE/180*M_PI)
					)
					continue;
				//calculate neighbor node value, if its smaller, update
				value_temp = sqrt(fabs(i-index_x)*fabs(i-index_x)+fabs(j-index_y)*fabs(j-index_y));
				// ROS_INFO("index_temp: %d, value_temp: %.3f, value_set[index_temp]: %.3f\n",index_temp,value_temp,value_set[index_temp]);
				unsettled_set.push_back(index_temp); //add neighbor node to unsettled set

				if(value_set[index_temp]>value_temp){
					// ROS_INFO("update enable i: %d, j: %d\n",i,j);
					value_set[index_temp] = value_temp;
					last_node_set[index_temp] = current_index;
				}
			}
		}
		// unsettled_set.erase(remove_if(unsettled_set.begin(), unsettled_set.end(), isP), unsettled_set.end());
		// unsettled_set.erase(remove_if(unsettled_set.begin(), unsettled_set.end(), current_index), unsettled_set.end());
		// std::cout << "Unsettled BEFORE DELETE, SIZE: " << unsettled_set.size() << " | ";
		// for(k = 0;k < unsettled_set.size();++k)
		// 	std::cout << unsettled_set[k] << " ";
		// std::cout << std::endl;

		//remove current node from unsettled and add to settled
		std::vector<int>::iterator invalid;
		invalid = std::remove( unsettled_set.begin(), unsettled_set.end(), current_index );
		unsettled_set.erase( invalid, unsettled_set.end() );
		settled_set.push_back(current_index);

		// std::cout << "Unsettled AFTER DELETE,  SIZE: " << unsettled_set.size() << " | ";
		// for(k = 0;k < unsettled_set.size();++k)
		// 	std::cout << unsettled_set[k] << " ";
		// std::cout << std::endl;

		//choose lowest value node from unsettled set, set it to current node
		value_temp1 = value_set[unsettled_set[0]];
		index_temp1 = unsettled_set[0];
		for(i = 0;i < unsettled_set.size();++i){
			if(value_set[unsettled_set[i]]<value_temp1){
				value_temp1 = value_set[unsettled_set[i]];
				index_temp1 = unsettled_set[i];
			}
		}

		current_index = index_temp1;
		index_x = current_index % global_map->size_x;
		index_y = current_index / global_map->size_x;

		visited_status_set[current_index] = 1;
		// ROS_INFO("settled_set size: %ld, unsettled_set size: %ld\n",settled_set.size(),unsettled_set.size());

	}

	ROS_WARN("WHILE LOOP DONE >>>>>>>>\n");
	//find the shortest path
	int index_x_goal = floor((end_pose.v[0] - global_map->origin_x) / global_map->scale + 0.5);
	int index_y_goal = floor((end_pose.v[1] - global_map->origin_x) / global_map->scale + 0.5);
	int index_goal = index_x_goal + index_y_goal * global_map->size_x;
	current_index = index_goal;
	index_x = current_index % global_map->size_x;
	index_y = current_index / global_map->size_x;

	std::vector<int> path_node_index_set;
	path_node_index_set.push_back(current_index);
	pose_vector temp_pose;
	temp_pose.v[0] = global_map->origin_x + ((index_x) - global_map->size_x / 2) * global_map->scale; //real world coord!!!!(according to car)
	temp_pose.v[1] = global_map->origin_y + ((index_y) - global_map->size_y / 2) * global_map->scale; //real world coord!!!!(according to car)
	temp_pose.v[2] = 0;
	std::vector<pose_vector> path_node_set;
	path_node_set.push_back(temp_pose);
	
	while(current_index != start_index){
		current_index = last_node_set[current_index];
		path_node_index_set.insert(path_node_index_set.begin(), current_index);
		index_x = current_index % global_map->size_x;
		index_y = current_index / global_map->size_x;
		temp_pose.v[0] = global_map->origin_x + ((index_x) - global_map->size_x / 2) * global_map->scale; //real world coord!!!!(according to car)
		temp_pose.v[1] = global_map->origin_y + ((index_y) - global_map->size_y / 2) * global_map->scale; //real world coord!!!!(according to car)
		temp_pose.v[2] = 0;
		path_node_set.insert(path_node_set.begin(), temp_pose);
	}
	ROS_INFO("FINAL PATH SIZE: %ld\n", path_node_set.size() );

	particlecloud_publisher(particlecloud_pub_, path_node_set);

}

bool motion_plan_processor1(pose_vector start_pose, pose_vector end_pose, ros::Publisher particlecloud_pub_){
	std::vector<int> unsettled_set;
	std::vector<int> settled_set;
	std::vector<bool> visited_status_set;
	std::vector<double> value_set;
	std::vector<int> last_node_set;

	int i,j,k,current_index;
	int index_temp,index_temp1;
	double value_temp,value_temp1;
	bool reachable_flag = 0;
	
	double inf=1.0/0.0;
	int totalNumber = index_set.size();
	ROS_INFO("total valid node number: %d\n",totalNumber);
	last_node_set.reserve(totalNumber);
	last_node_set.resize(totalNumber);
	value_set.reserve(totalNumber);
	value_set.resize(totalNumber,inf); //set all nodes' value to inf
	visited_status_set.reserve(totalNumber);
	visited_status_set.resize(totalNumber,0);

	ROS_INFO("inf is %lf\n",inf);

	int index_x = floor((start_pose.v[0] - global_map->origin_x) / global_map->scale + 0.5);
	int index_y = floor((start_pose.v[1] - global_map->origin_x) / global_map->scale + 0.5);
	int start_index = index_x + index_y * global_map->size_x;
	ROS_INFO("index_x : %d, index_y : %d, current_index: %d\n",index_x,index_y,start_index);
	//set start node as current node
	for(i = 0;i < index_set.size();++i)
	{
		if(index_set[i] == start_index && global_map->cells[index_set[i]].occ_dist >= MAX_OCC_DIST)
		{
			current_index = i;
			break;
		}
		else if(i == index_set.size()-1)
		{
			ROS_ERROR("CURRENT POINT(START) NOT FOUND!!!!\n");
			return 0;
		}
	}

	value_set[current_index] = 0; //set current node's value to 0;

	unsettled_set.push_back(current_index); //add current node to unsettled set
	visited_status_set[current_index] = 1; //set current node's visited status to 1

	// ROS_INFO("isP = %d",isP(1,1));

	// ROS_INFO("compareP = %d",compareP(inf,1));

	ROS_WARN("ENTERING WHILE LOOP >>>>>>>>\n");

	// main while loop
	while(unsettled_set.size() != 0){
		for(i = 0;i < totalNumber;++i){

				// ROS_INFO("i: %d, j: %d\n",i,j);
				index_temp = index_set[i];
				//the neighbor node has to be free and can not be itself and must be unvisited and must
				if((global_map->cells[index_temp].occ_state != -1)||visited_status_set[i] == 1|| global_map->cells[index_temp].occ_dist < MAX_OCC_DIST)
					continue;
				//calculate neighbor node value, if its smaller, update
				value_temp = distance_calculator(sample_set[current_index], sample_set[i]);
				// ROS_INFO("index_temp: %d, value_temp: %.3f, value_set[index_temp]: %.3f\n",index_temp,value_temp,value_set[index_temp]);
				//see if it's a neighbor or not
				if(value_temp < inf){
					unsettled_set.push_back(i); //add neighbor node to unsettled set
					if(value_set[i]>value_temp){
						// ROS_INFO("update enable i: %d, j: %d\n",i,j);
						value_set[i] = value_temp;
						last_node_set[i] = current_index;
					}
				}
		}

		// std::cout << "Unsettled BEFORE DELETE, SIZE: " << unsettled_set.size() << " | ";
		// for(k = 0;k < unsettled_set.size();++k)
		// 	std::cout << unsettled_set[k] << " ";
		// std::cout << std::endl;

		//remove current node from unsettled and add to settled
		std::vector<int>::iterator invalid;
		invalid = std::remove( unsettled_set.begin(), unsettled_set.end(), current_index );
		unsettled_set.erase( invalid, unsettled_set.end() );
		settled_set.push_back(current_index);

		// std::cout << "Unsettled AFTER DELETE,  SIZE: " << unsettled_set.size() << " | ";
		// for(k = 0;k < unsettled_set.size();++k)
		// 	std::cout << unsettled_set[k] << " ";
		// std::cout << std::endl;

		//choose lowest value node from unsettled set, set it to current node
		value_temp1 = value_set[0];
		index_temp1 = index_set[0];
		for(i = 0;i < unsettled_set.size();++i){
			if(value_set[unsettled_set[i]]<value_temp1){
				value_temp1 = value_set[unsettled_set[i]];
				index_temp1 = index_set[unsettled_set[i]];
			}
		}

		current_index = index_temp1;
		index_x = index_set[current_index] % global_map->size_x;
		index_y = index_set[current_index] / global_map->size_x;

		visited_status_set[current_index] = 1;
		ROS_INFO("settled_set size: %ld, unsettled_set size: %ld\n",settled_set.size(),unsettled_set.size());

	}

	ROS_WARN("WHILE LOOP DONE >>>>>>>>\n");
	//find the shortest path
	int index_x_goal = floor((end_pose.v[0] - global_map->origin_x) / global_map->scale + 0.5);
	int index_y_goal = floor((end_pose.v[1] - global_map->origin_x) / global_map->scale + 0.5);
	int index_goal = index_x_goal + index_y_goal * global_map->size_x;
	//set end node as current node
	for(i = 0;i < index_set.size();++i)
	{
		if(index_set[i] == index_goal && global_map->cells[i].occ_dist >= MAX_OCC_DIST)
		{
			current_index = i;
			break;
		}
		else if(i == index_set.size()-1)
		{
			ROS_ERROR("CURRENT POINT(GOAL) NOT FOUND!!!!\n");
			return 0;
		}
	}

	index_x = index_x_goal;
	index_y = index_y_goal;

	std::vector<int> path_node_index_set;
	path_node_index_set.push_back(current_index);
	pose_vector temp_pose;
	temp_pose.v[0] = global_map->origin_x + ((index_x) - global_map->size_x / 2) * global_map->scale; //real world coord!!!!(according to car)
	temp_pose.v[1] = global_map->origin_y + ((index_y) - global_map->size_y / 2) * global_map->scale; //real world coord!!!!(according to car)
	temp_pose.v[2] = 0;
	std::vector<pose_vector> path_node_set;
	path_node_set.push_back(temp_pose);
	
	while(index_set[current_index] != index_set[start_index]){
		current_index = last_node_set[current_index];
		path_node_index_set.insert(path_node_index_set.begin(), index_set[current_index]);
		index_x = index_set[current_index] % global_map->size_x;
		index_y = index_set[current_index] / global_map->size_x;
		temp_pose.v[0] = global_map->origin_x + ((index_x) - global_map->size_x / 2) * global_map->scale; //real world coord!!!!(according to car)
		temp_pose.v[1] = global_map->origin_y + ((index_y) - global_map->size_y / 2) * global_map->scale; //real world coord!!!!(according to car)
		temp_pose.v[2] = 0;
		path_node_set.insert(path_node_set.begin(), temp_pose);
	}
	ROS_INFO("FINAL PATH SIZE: %ld\n", path_node_set.size() );

	particlecloud_publisher(particlecloud_pub_, path_node_set);

	return 1;

}

int main(int argc, char *argv[])
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	int i,j;
	angle_ref_set.reserve(3);
	angle_ref_set.resize(3);
	for(i = 0;i < 3;++i)
	{
		angle_ref_set[i].reserve(3);
		angle_ref_set[i].resize(3);
	}
	angle_ref_set[0][0] = -135/180*M_PI;	
	angle_ref_set[0][1] = 180/180*M_PI;
	angle_ref_set[0][2] = 135/180*M_PI;
	angle_ref_set[1][0] = -90/180*M_PI;
	angle_ref_set[1][2] = 90/180*M_PI;
	angle_ref_set[2][0] = -45/180*M_PI;
	angle_ref_set[2][1] = 0/180*M_PI;
	angle_ref_set[2][2] = 45/180*M_PI;

	ros::init(argc, argv, "motion_plan");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	pose_vector start_pose;
	pose_vector end_pose;

	ros::Publisher particlecloud_pub_;

	ros::Rate loop_rate(1);

	mapRequest();

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	int count = 0;
	while (ros::ok())
	{

		ros::spinOnce();

		start_pose.v[0] = 0 + (global_map->size_x / 2) * global_map->scale;
		start_pose.v[1] = 0 + (global_map->size_y / 2) * global_map->scale;
		start_pose.v[2] = 0;

		end_pose.v[0] = 3.2 + (global_map->size_x / 2) * global_map->scale;
		end_pose.v[1] = -3.6 + (global_map->size_y / 2) * global_map->scale;
		end_pose.v[2] = -65/180*M_PI;

		ROS_INFO("start_pose: %.3f, %.3f, %.3f\n",start_pose.v[0],start_pose.v[1],start_pose.v[2]);
		ROS_INFO("end_pose  : %.3f, %.3f, %.3f\n",end_pose.v[0],end_pose.v[1],end_pose.v[2]);

		particlecloud_pub_ = n.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);
		// particlecloud_publisher(particlecloud_pub_,start_pose,end_pose);

		if(method_switch == 0)
		{		
			motion_plan_processor(start_pose, end_pose, particlecloud_pub_);
		}
		else if(method_switch == 1){
			bool result = motion_plan_processor1(start_pose, end_pose, particlecloud_pub_);
		}

		loop_rate.sleep();
		++count;
		break;
	}

	free(global_map->cells);
	free(global_map);

	return 0;
}