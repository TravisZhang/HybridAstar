#include "hybrid_astar/map.h"

namespace ha
{

// Create a new map
inline map_t* Map::map_alloc(void)
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

Map::Map()
{
	global_map = map_alloc();
	ROS_ASSERT(global_map);
	cost_map = map_alloc();
	ROS_ASSERT(cost_map);
}
Map::~Map()
{
	free(global_map->cells);
	free(global_map);
	global_map = NULL;
	free(cost_map->cells);
	free(cost_map);
	cost_map = NULL;
}

void Map::mapRequest(){

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

void Map::mapReceived(const nav_msgs::OccupancyGrid& grid)
{
	ROS_INFO("Received a %d X %d map @ %.3f m/pix, Origin x: %.3f, y: %.3f \n",
           grid.info.width,
           grid.info.height,
           grid.info.resolution,
		   grid.info.origin.position.x,
		   grid.info.origin.position.y);

  	global_map->size_x = grid.info.width;
  	global_map->size_y = grid.info.height;
  	global_map->scale = grid.info.resolution;
  	global_map->origin_x = grid.info.origin.position.x + (global_map->size_x / 2) * global_map->scale; //prevent negative coords
  	global_map->origin_y = grid.info.origin.position.y + (global_map->size_y / 2) * global_map->scale; //prevent negative coords
  	// Convert to player format
	//  The map data, in row-major order, starting with (0,0).  Occupancy
	//  probabilities are in the range [0,100].  Free is 0.
  	// global_map->cells = (map_cell_t**)malloc(sizeof(map_cell_t*)*global_map->size_x);
	// for(int i=0;i<global_map->size_x;++i)
	// {
	// 	global_map->cells[i] = (map_cell_t*)malloc(sizeof(map_cell_t)*global_map->size_y);
	// }
	global_map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*global_map->size_x*global_map->size_y);
  	ROS_ASSERT(global_map->cells);
  	for(int i=0;i<global_map->size_x * global_map->size_y;i++)
  	{
    	if(grid.data[i] == 0)
      		global_map->cells[i].occ_state = 0;
    	// else if(grid.data[i] == 100)
      	// 	global_map->cells[i].occ_state = +1;
    	else if(grid.data[i] >= 65)
      		global_map->cells[i].occ_state = 1;
  	}

}

void Map::mapFromPic(std::string obs_path)
{
	global_map->size_x = 800;
  	global_map->size_y = 800;
  	global_map->scale = 1;
  	global_map->origin_x = 0; 
  	global_map->origin_y = 0;
	int index_temp = 0;
	ROS_INFO("global map size: %d x %d\n",global_map->size_x,global_map->size_y);

	ROS_INFO("reading map!!!!!\n");
	Mat obsmap=imread(obs_path, 0);
	if(obsmap.empty() == true)
	{
		ROS_ERROR("map reading failed!!!!!! EXIT\n");
		exit(1);
	}
	ROS_INFO("obsmap size: %d x %d\n",obsmap.rows,obsmap.cols);
	imshow("obsmap", obsmap);
	waitKey(0);

	global_map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*global_map->size_x*global_map->size_y);
  	ROS_ASSERT(global_map->cells);

	pub_obs_map_ptr = boost::make_shared<nav_msgs::OccupancyGrid>();
	pub_obs_map_ptr->header.seq =1;
	pub_obs_map_ptr->header.frame_id = "map";
	pub_obs_map_ptr->info.height = global_map->size_x;
	pub_obs_map_ptr->info.width = global_map->size_y;
	pub_obs_map_ptr->info.resolution = global_map->scale;
	pub_obs_map_ptr->info.origin.position.x = global_map->origin_x;
	pub_obs_map_ptr->info.origin.position.y = global_map->origin_y;
	pub_obs_map_ptr->data.clear();
	pub_obs_map_ptr->data.resize(global_map->size_x*global_map->size_y,0);

	Mat dist(global_map->size_x, global_map->size_y, CV_8UC3, Scalar(255, 255, 255)); //CV_8UC3:8 Units 3 Channel Matrix
	double scale = global_map->size_x/obsmap.rows;

	// convert 400*400 img into 800*800 obs_map
	for(int j=0;j<global_map->size_y;j++)
		for(int i=0;i<global_map->size_x;i++)		
		{
			index_temp = i + j * global_map->size_x;
			if(obsmap.at<uchar>((global_map->size_y-j-1)/scale,i/scale)<=120)
			{
				global_map->cells[index_temp].occ_state = 1; 
			}
			else
			{
				global_map->cells[index_temp].occ_state = 0;
			}
			dist.at<Vec3b>((global_map->size_y-j-1),i)={255-255*global_map->cells[index_temp].occ_state, 
								200-200*global_map->cells[index_temp].occ_state, 
								200-200*global_map->cells[index_temp].occ_state};
			pub_obs_map_ptr->data[index_temp] = global_map->cells[index_temp].occ_state * 100;
		}
	// resize(dist, dist, Size(400, 400));
	//uncomment to check if dijkstra ran properly
	imshow("globalmap", dist);
	waitKey(0);
	ROS_INFO("reading map finished!!!!!\n");
}

bool Map::isBoundary(int i0, int j0)
{
	int i,j,index_temp;
	for(j = j0-1;j <= j0+1;++j)
	{
		for(i = i0-1;i <= i0+1;++i)
		{
			if(i < 0 || i >= global_map->size_x || j < 0 || j >= global_map->size_y)
				continue;
			if(i == i0 || j == j0)
				continue;
			index_temp = i + j * global_map->size_x;
			if(global_map->cells[index_temp].occ_state == 0)	
				return true;
		}
	}
	return false;
}

void Map::obsUpdate()
{
	int i,j,index_temp;
	// find all boundary cells and mark all cells inside obs as -1
	std::queue<int> index_q;
	for(j = 0;j < global_map->size_y;++j)
	{
		for(i = 0;i < global_map->size_x;++i)
		{
			index_temp = i + j * global_map->size_x;
			if(global_map->cells[index_temp].occ_state == 1)
			{
				if(isBoundary(i,j))
				{
					global_map->cells[index_temp].occ_dist = 1;
					index_q.push(index_temp);
				}
				else
				{
					global_map->cells[index_temp].occ_dist = -1;
				}
			}
		}
	}

	// update occ_dist for all free cells
	int i0,j0,index_temp0;
	while(!index_q.empty())
	{
		index_temp0 = index_q.front();
		index_q.pop();
		i0 = index_temp0 % global_map->size_x;
		j0 = index_temp0 / global_map->size_x;
		for(j = j0-1;j <= j0+1;++j)
		{
			for(i = i0-1;i <= i0+1;++i)
			{
				index_temp = i + j * global_map->size_x;
				if(i < 0 || i >= global_map->size_x || j < 0 || j >= global_map->size_y)
					continue;
				if(i == i0 || j == j0)
					continue;
				if(global_map->cells[index_temp].occ_state == 0 && global_map->cells[index_temp].occ_dist == 0)
				{
					global_map->cells[index_temp].occ_dist = global_map->cells[index_temp0].occ_dist + 1;
					index_q.push(index_temp);
				}
			}
		}
	}

	max_dist_index = index_temp;
	obs_init_flag = true;

	// //display the obs dist map
	Mat dist0(global_map->size_x, global_map->size_y, CV_8UC3, Scalar(255, 255, 255)); //CV_8UC3:8 Units 3 Channel Matrix
	for(j=0;j<global_map->size_y;j++)
		for(i=0;i<global_map->size_x;i++)
		{
			index_temp = i + j * global_map->size_x;
			int obsdist = int(global_map->cells[index_temp].occ_dist);
			if(obsdist == -1) obsdist = 10;
			dist0.at<Vec3b>((global_map->size_y-j-1),i)={255-0.3*obsdist, 255-0.3*obsdist, 255-0.3*obsdist};
		}
	// resize(dist0, dist0, Size(400, 400));
	//uncomment to check if dijkstra ran properly
	imshow("obsdist", dist0);
	waitKey(0);
}

void Map::cost_map_init()
{

	cost_map->size_x = DX;
  	cost_map->size_y = DY;
  	cost_map->scale = global_map->size_x/DX*global_map->scale;
  	cost_map->origin_x = global_map->origin_x; 
  	cost_map->origin_y = global_map->origin_y; 

	ROS_INFO("cost map size: %d x %d\n",cost_map->size_x,cost_map->size_y);

	cost_map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*cost_map->size_x*cost_map->size_y);
  	ROS_ASSERT(cost_map->cells);

	// int index_temp = 0;
	// int temp_gx,temp_gy;
	// for(int i = 0;i < cost_map->size_x * cost_map->size_y;i++)
  	// {
	// 	// ROS_INFO("i : %d",i);
	// 	temp_gx = (i % cost_map->size_x) * cost_map->scale / global_map->scale;
	// 	temp_gy = (i / cost_map->size_x) * cost_map->scale / global_map->scale;
	// 	temp_gx = std::max(0,temp_gx);
	// 	temp_gx = std::min(global_map->size_x-1,temp_gx);
	// 	temp_gy = std::max(0,temp_gy);
	// 	temp_gy = std::min(global_map->size_y-1,temp_gy);
	// 	index_temp = temp_gx + temp_gy * global_map->size_x;
	// 	// ROS_INFO("index_temp : %d",index_temp);
    // 	if(global_map->cells[index_temp].occ_state == 1)
	// 	{
	// 		cost_map->cells[i].occ_state = 1;
	// 	}
    // 	else
	// 	{
	// 		cost_map->cells[i].occ_state = 0;
	// 	} 		
  	// }

}

void Map::runAstar(Pose_vector start_pose, Pose_vector end_pose)
{
	int i,j,index_temp;
	int node_status[cost_map->size_x][cost_map->size_y] = {0}; //0: initial 1: in openset 2: in closeset
	std::priority_queue<Pose_vector, std::vector<Pose_vector>, cmp0> openset; // node discovered not yet evaluated
	Pose_vector target,current,neighbor;
	target.dx = int((end_pose.v[0] - global_map->origin_x) / cost_map->scale); // x index in cost_map
	target.dy = int((end_pose.v[1] - global_map->origin_y) / cost_map->scale); // y index in cost_map
	ROS_INFO("target dx: %d, dy: %d\n",target.dx,target.dy);

	target.dx = std::max(0,target.dx);
	target.dx = std::min(cost_map->size_x-1,target.dx);
	target.dy = std::max(0,target.dy);
	target.dy = std::min(cost_map->size_y-1,target.dy);
	// initialize cost_map & target cost
	for(i=0;i<cost_map->size_x * cost_map->size_y;i++)
  	{
      	cost_map->cells[i].cost = 100000;
  	}

	index_temp = target.dx + target.dy * cost_map->size_x;
	cost_map->cells[index_temp].cost = 0;
	target.gscore = 0;
	target.fscore = 0;

	openset.push(target); 
	// node_status[target.dx][target.dy] = 1;

	max_cost2d = 0;

	while(!openset.empty())
	{
		// smallest fscore cost node -> current
		current = openset.top();
		// delete current from openset
		openset.pop();

		if(node_status[current.dx][current.dy] == 1)
			continue;

		// add to close set
		// node_status[current.dx][current.dy] = 2;
		node_status[current.dx][current.dy] = 1;
		for(j = -1;j <= 1;++j)
		{
			for(i = -1;i <= 1;++i)
			{
				// can not evaluate current itself
				if(i == 0 && j == 0)
					continue;

				// check if out of cost_map and if in closeset
				neighbor.dx = current.dx+i;
				neighbor.dy = current.dy+j;
				// if(	(node_status[neighbor.dx][neighbor.dy] == 2) || 
				if(	(neighbor.dx < 0 || neighbor.dx >= cost_map->size_x) ||
					(neighbor.dy < 0 || neighbor.dy >= cost_map->size_y))
				{
					continue;
				}

				// neighbor can not be occupied
				int temp_gx = neighbor.dx * cost_map->scale / global_map->scale;
				int temp_gy = neighbor.dy * cost_map->scale / global_map->scale;
				temp_gx = std::max(0,temp_gx);
				temp_gx = std::min(global_map->size_x-1,temp_gx);
				temp_gy = std::max(0,temp_gy);
				temp_gy = std::min(global_map->size_y-1,temp_gy);
				index_temp = temp_gx + temp_gy * global_map->size_x;
				if(global_map->cells[index_temp].occ_state != 0)
					continue;
				// index_temp = neighbor.dx + neighbor.dy * cost_map->size_x;
				// if(cost_map->cells[index_temp].occ_state != 0)
				// 	continue;

				// if newly discovered not yet evaluated
				// if(node_status[neighbor.dx][neighbor.dy] == 0)
				// {
					// node_status[neighbor.dx][neighbor.dy] = 1;
					index_temp = current.dx + current.dy * cost_map->size_x;
					neighbor.gscore = cost_map->cells[index_temp].cost + sqrt(i*i+j*j);
					neighbor.fscore = neighbor.gscore + abs(target.dx-neighbor.dx) + abs(target.dy-neighbor.dy);
					index_temp = neighbor.dx + neighbor.dy * cost_map->size_x;
					// update cost
					if(neighbor.gscore < cost_map->cells[index_temp].cost)
						cost_map->cells[index_temp].cost = neighbor.gscore;
					openset.push(neighbor);
					max_cost2d = std::max(max_cost2d,cost_map->cells[index_temp].cost);
				// }
			}
		}

	}

	ROS_INFO("max_cost2d : %.3f\n",max_cost2d);

}

bool Map::checkCollision(Pose_vector current)
{
	// if(obs_init_flag == false)
	// 	return true;
	
	double i,j,x0,y0,x,y,heading;
	double delta = DELTA_CHECK;
	int gx,gy,index_temp;
	heading = current.v[2]/180*M_PI;
	for(i = CAR_LENGTH/2;i >= -CAR_LENGTH/2;i -= delta)
	{
		for(j = -CAR_WIDTH/2; j <= CAR_WIDTH/2; j += delta)
		{
			if(i < CAR_LENGTH/2 && i > -CAR_LENGTH/2 && j < CAR_WIDTH/2 && j > -CAR_WIDTH/2)
				continue;
			x = current.v[0] + i*cos(heading) - j*sin(heading);
			y = current.v[1] + i*sin(heading) + j*cos(heading);
			gx = floor((x - global_map->origin_x) / global_map->scale + 0.5);
			gy = floor((y - global_map->origin_x) / global_map->scale + 0.5);
			if(gx < 0 || gx >= global_map->size_x || gy < 0 || gy >= global_map->size_y)
				return true;
			index_temp = gx + gy * global_map->size_x;
			if(global_map->cells[index_temp].occ_state == 1)
				return true;
			else if(global_map->cells[index_temp].occ_dist < OBS_DIST_MIN/global_map->scale)
				return true;
		}
	}

	return false;
}



void Map::free_map(int type)
{
	switch (type)
	{
		case 1:
			free(global_map->cells);
			free(global_map);
			break;
		case 2:
			free(cost_map->cells);
			free(cost_map);
			break;
		default:
			free(global_map->cells);
			free(global_map);
			free(cost_map->cells);
			free(cost_map);
	}
	
}

}
