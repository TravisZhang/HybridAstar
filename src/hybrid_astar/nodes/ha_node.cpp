#include "ha_node.h"

namespace ha
{

Ha_node::Ha_node()
{

	initialize();
	
}

void Ha_node::initialize(void)
{
	particlecloud_pub_ = nh.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);
	global_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("gridmap", 1,true);

	// ha_map.mapRequest();

	path = ros::package::getPath("hybrid_astar");
	ROS_INFO("Package path: %s\n",path.c_str());
	obs_path = path+"/maps/map.png";

	ha_map.mapFromPic(obs_path);
	pub_obs_map = *(ha_map.pub_obs_map_ptr);

	GRID_RES = ha_map.global_map->size_x/GX;
	ROS_INFO("GRID_RES : %d\n",GRID_RES);

	if(ha_map.global_map)
	{
		// start_pose.v[0] = 0 + (ha_map.global_map->size_x / 2) * ha_map.global_map->scale;
		// start_pose.v[1] = 0 + (ha_map.global_map->size_y / 2) * ha_map.global_map->scale;
		// start_pose.v[2] = 0;

		// end_pose.v[0] = 3.2 + (ha_map.global_map->size_x / 2) * ha_map.global_map->scale;
		// end_pose.v[1] = -3.6 + (ha_map.global_map->size_y / 2) * ha_map.global_map->scale;
		// end_pose.v[2] = -65;

		// coords in normal frame(y up x right)
		start_pose.v[0] = 700 ;
		start_pose.v[1] = 699 ; //100
		start_pose.v[2] = 225; //135

		end_pose.v[0] = 100 ;
		end_pose.v[1] = 199 ; //600
		end_pose.v[2] = 270; //90

	}

	ROS_INFO("start_pose: %.3f, %.3f, %.3f\n",start_pose.v[0],start_pose.v[1],start_pose.v[2]);
	ROS_INFO("end_pose  : %.3f, %.3f, %.3f\n",end_pose.v[0],end_pose.v[1],end_pose.v[2]);

	// std::vector<Pose_vector> next_pose;
	// next_pose = start_pose.getNextPose();
}

void Ha_node::process(void)
{
	ros::Rate loop_rate(loop_freq);
	
	while(nh.ok())
	{

		ROS_INFO("222");
		ros::spinOnce();

		hybridAstar();

		particlecloud_publisher(particlecloud_pub_, final_path);
		// particlecloud_publisher(particlecloud_pub_, current_set);


		global_map_pub_.publish(pub_obs_map);
		ROS_INFO("publish global map done!!!\n");

		loop_rate.sleep();

		break;

	}

	// ha_map.free_map(3);
}

void Ha_node::hybridAstar(void)
{
	ROS_INFO("Begin  obstacle update >>>>>>>>>>\n");
	ha_map.obsUpdate();
	ROS_INFO("Finish obstacle update >>>>>>>>>>\n");

	ROS_INFO("Begin  costmap init >>>>>>>>>>\n");
	ha_map.cost_map_init();
	ROS_INFO("Finish costmap init >>>>>>>>>>\n");

	ROS_INFO("Begin  costmap update >>>>>>>>>>\n");
	ha_map.runAstar( start_pose, end_pose);
	ROS_INFO("Finish costmap update >>>>>>>>>>\n");
	global_cost_map = ha_map.cost_map;
	global_obs_map = ha_map.global_map;
	// global_cost_map(ha_map.cost_map);

	// //display the cost map
	Mat dist1(ha_map.cost_map->size_x, ha_map.cost_map->size_y, CV_8UC3, Scalar(255, 255, 255)); //CV_8UC3:8 Units 3 Channel Matrix
	for(int j=0;j<ha_map.cost_map->size_y;j++)
		for(int i=0;i<ha_map.cost_map->size_x;i++)
		{
			int index_temp = i + j * ha_map.cost_map->size_x;
			int cost2d = int(ha_map.cost_map->cells[index_temp].cost);
			if(cost2d > 9999) cost2d = 510;
			dist1.at<Vec3b>((ha_map.cost_map->size_y-j-1),i)={255-0.5*cost2d, 255-0.5*cost2d, 255-0.5*cost2d};
		}
	// resize(dist1, dist1, Size(400, 400));
	//uncomment to check if dijkstra ran properly
	imshow("cost2d", dist1);
	waitKey(0);


	std::cout << "yyy" << std::endl;
	
	std::priority_queue<Pose_vector, std::vector<Pose_vector>, cmp1> poseset;

	start_pose.cost3d = 0;

	std::cout << "xxx" << std::endl;

	// vis = new int**[GX];
	// for(int i = 0;i < GX;++i)
	// {
	// 	vis[i] = new int*[GY];
	// 	for(int j = 0;j < GY;++j)
	// 	{
	// 		vis[i][j] = new int[THETA];
	// 		// for(int k = 0;k < THETA;++k)
	// 		// {
	// 		// 	vis[i][j][k] = 0;
	// 		// }
	// 	}
	// }
	// memset(vis, 0, sizeof(int)*GX*GY*THETA);

	vis.clear();
	vis.resize(GX);
	for(int i = 0;i < GX;++i)
	{
		vis[i].resize(GY);
		for(int j = 0;j < GY;++j)
		{
			vis[i][j].resize(THETA,0);
		}
	}

	// previous_set.clear();
	// previous_set.resize(GX);
	// for(int i = 0;i < GX;++i)
	// {
	// 	previous_set[i].resize(GY);
	// 	for(int j = 0;j < GY;++j)
	// 	{
	// 		previous_set[i][j].resize(THETA);
	// 	}
	// }

	Pose_vector current;
	std::vector<Pose_vector> nextset;
	current_set.clear();

	// std::cout << "aaa" << std::endl;

	// start_pose.gx = int((start_pose.v[0] - ha_map.global_map->origin_x) / GRID_RES);
	// start_pose.gy = int((start_pose.v[1] - ha_map.global_map->origin_x) / GRID_RES);
	start_pose.gx = int((start_pose.v[0] - ha_map.global_map->origin_x) / ha_map.global_map->scale / ha_map.global_map->size_x * GX );
	start_pose.gy = int((start_pose.v[1] - ha_map.global_map->origin_x) / ha_map.global_map->scale / ha_map.global_map->size_y * GY );
	start_pose.gtheta = int(start_pose.v[2]/THETA_RES);
	while(start_pose.gtheta < 0)
	{
		start_pose.gtheta += THETA;
	}
	while(start_pose.gtheta > THETA)
	{
		start_pose.gtheta -= THETA;
	}
	start_pose.start_flag = 1;

	// std::cout << "bbb" << std::endl;

	// end_pose.gx = int((end_pose.v[0] - ha_map.global_map->origin_x) / GRID_RES);
	// end_pose.gy = int((end_pose.v[1] - ha_map.global_map->origin_x) / GRID_RES);
	end_pose.gx = int((end_pose.v[0] - ha_map.global_map->origin_x) / ha_map.global_map->scale / ha_map.global_map->size_x * GX );
	end_pose.gy = int((end_pose.v[1] - ha_map.global_map->origin_x) / ha_map.global_map->scale / ha_map.global_map->size_y * GY );
	end_pose.gtheta = int(end_pose.v[2]/THETA_RES);
	while(end_pose.gtheta < 0)
	{
		end_pose.gtheta += THETA;
	}
	while(end_pose.gtheta > THETA)
	{
		end_pose.gtheta -= THETA;
	}

	ROS_INFO("start_pose g: %d, %d, %d\n",start_pose.gx,start_pose.gy,start_pose.gtheta);
	ROS_INFO("end_pose   g: %d, %d, %d\n",end_pose.gx,end_pose.gy,end_pose.gtheta);

	// std::cout << "ccc" << std::endl;

	poseset.push(start_pose);

	int iter = 0;
	ROS_WARN("Enter main loop >>>>>>>>>>>>>>>>>>>>>\n");
	while(!poseset.empty())
	{
		ROS_INFO("************ iter : %d *************\n",iter);
		current = poseset.top();
		poseset.pop();
		ROS_INFO("current g: %d, %d, %d\n",current.gx,current.gy,current.gtheta);
		ROS_INFO("target  g: %d, %d, %d\n",end_pose.gx,end_pose.gy,end_pose.gtheta);
		if(abs(current.gx-end_pose.gx)<=1 && abs(current.gy-end_pose.gy)<=1 && abs(current.gtheta-end_pose.gtheta)<=2)
		{
			ROS_INFO("Goal Reached !!!!!!!\n");
			path_valid = 1;
			break;
		}

		// if(init_flag == 0)
		// {
		// 	head = &(current);
		// 	head->previous = NULL;
		// 	head->start_flag = 1;
		// 	init_flag = 1;
		// }

		if(vis[current.gx][current.gy][current.gtheta] == 1)
		{
			if(poseset.size()<=0)
			{
				ROS_ERROR("no path found!!!!!!\n");
				path_valid = 0;
				break;
			}
			ROS_INFO("visited continue!!!!!!\n");
			continue;
		}

		vis[current.gx][current.gy][current.gtheta] = 1;

		ROS_INFO("current start_flag: %d\n", current.start_flag);

		nextset.clear();
		nextset = current.getNextPose();	
		ROS_INFO("---------- nextset size: %ld ----------\n",nextset.size());
		// particlecloud_publisher(particlecloud_pub_, nextset);

		current_set.push_back(current);
		for(int i = 0;i < nextset.size();++i)
		{
			ROS_INFO("i: %d",i);
			if(ha_map.checkCollision(nextset[i]))
			{
				ROS_WARN("collide with obs !!!!\n");
				continue;
			}		

			// nextset[i].gx = int((nextset[i].v[0] - ha_map.global_map->origin_x) / GRID_RES);
			// nextset[i].gy = int((nextset[i].v[1] - ha_map.global_map->origin_x) / GRID_RES);
			nextset[i].gx = int((nextset[i].v[0] - ha_map.global_map->origin_x) / ha_map.global_map->scale / ha_map.global_map->size_x * GX );
			nextset[i].gy = int((nextset[i].v[1] - ha_map.global_map->origin_x) / ha_map.global_map->scale / ha_map.global_map->size_y * GX );
			nextset[i].gtheta = int(nextset[i].v[2]/THETA_RES);
			while(nextset[i].gtheta < 0)
			{
				nextset[i].gtheta += THETA;
			}
			while(nextset[i].gtheta > THETA)
			{
				nextset[i].gtheta -= THETA;
			}
			if(vis[nextset[i].gx][nextset[i].gy][nextset[i].gtheta])
			{
				ROS_WARN("evaluated continue !!!!\n");
				continue;
			}	

			if(i == floor(nextset.size()/2))
			// if(i == 1)
			{
				nextset[i].cost3d = current.cost3d + 5;
			}
			else
			{
				nextset[i].cost3d = current.cost3d + 15;
			}

			ROS_INFO("heading_next: %.3f",nextset[i].v[2]);
        	ROS_INFO("x_next: %.3f, y_next: %.3f, h_next: %.3f\n",nextset[i].v[0],nextset[i].v[1],nextset[i].v[2]);

			// previous_set[nextset[i].gx][nextset[i].gy][nextset[i].gtheta] = current;
			// nextset[i].previous = &(current);
			// nextset[i].After(&(current));
			// nextset[i].pprevious = current;
			// Insert(head, &(nextset[i]));
			nextset[i].previous_num = current_set.size()-1;

			nextset[i].dx = int((nextset[i].v[0] - ha_map.global_map->origin_x) / ha_map.cost_map->scale );
			nextset[i].dy = int((nextset[i].v[1] - ha_map.global_map->origin_x) / ha_map.cost_map->scale );
			ROS_INFO("x_next d: %d, y_next: %d\n",nextset[i].dx,nextset[i].dy);

			poseset.push(nextset[i]);

		}
		

		ROS_INFO("************ iter end !!!!! poseset size: %ld ************\n",poseset.size());

		iter++;

	}

	if(path_valid == 1)
	{
		ROS_INFO("Begin  path reconstruct  >>>>>>>>>>\n");
		reconstruct_path(current);
		ROS_INFO("Finish path reconstruct  >>>>>>>>>>\n");
	}
	else
	{
		ROS_INFO("no path constructed!!!!!!\n");
	}

	// for(int i = 0;i < GX;++i)
	// {
	// 	for(int j = 0;j < GY;++j)
	// 	{
	// 		delete vis[i][j];
	// 	}
	// 	delete vis[i];
	// }
	// delete vis;


}

void Ha_node::reconstruct_path(Pose_vector current)
{
	final_path.clear();
	Pose_vector pose_temp;
	final_path.push_back(current);

	int counter = 0;
	// while(current.gx!=start_pose.gx || current.gy!=start_pose.gy || current.gtheta!=start_pose.gtheta)
	// while(current.start_flag != 1)
	// while(current.previous != NULL)
	while(current.previous_num != -1)
	{
		// pose_temp = previous_set[current.gx][current.gy][current.gtheta];
		// pose_temp = *current.previous;
		// pose_temp = current.pprevious;
		pose_temp = current_set[current.previous_num];
		current = pose_temp;
		final_path.push_back(current);
		ROS_INFO("counter: %d",counter);
		counter++;
		// if(counter>5) break;
	}
}

void Ha_node::particlecloud_publisher(ros::Publisher particlecloud_pub_, std::vector<Pose_vector> sample_set){
	// ros::Rate r(1/LOOP_DURATION);
	ROS_INFO("Pointcloud publish Begin >>>>>>>>>>>>> \n");
	geometry_msgs::PoseArray cloud_msg;
	std::vector< Pose_vector > pub_set;
	// if(sample_cloud_pub_flag == 0) pub_set = sample_set;
	// else pub_set = resample_set;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
    cloud_msg.poses.resize(sample_set.size());
    for(int i=0;i<sample_set.size();i++)
    {
    // tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(sample_set[i].v[2]/180*M_PI),
    //                             tf::Vector3(sample_set[i].v[0]-global_obs_map->size_x / 2 * global_obs_map->scale,
    //                                     sample_set[i].v[1]-global_obs_map->size_y / 2 * global_obs_map->scale, 0)),
    //                 cloud_msg.poses[i]);
    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(sample_set[i].v[2]/180*M_PI),
                                tf::Vector3(sample_set[i].v[0],
                                        sample_set[i].v[1], 0)),
                    cloud_msg.poses[i]);
    }
    particlecloud_pub_.publish(cloud_msg);
	ROS_INFO("Pointcloud publish Done >>>>>>>>>>>>> \n");
	// r.sleep();
}

}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "planner_node");

	ha::Ha_node ha_node;

	ha_node.process();

	return 0;
}