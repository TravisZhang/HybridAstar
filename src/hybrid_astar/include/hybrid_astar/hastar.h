#ifndef HSTAR_H
#define HSTAR_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include "bits/stdc++.h" // for std::queue

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer.hpp>

#include "color.h"

//ROS include
//ros map
#include <map>
#include "ros/ros.h"
#include "ros/assert.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

#include <ros/package.h>

// OpenCV
// #include <cv_bridge/cv_bridge.h>
// #include "opencv/cv.h"
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/opencv.hpp>

using namespace cv;

namespace ha
{
    extern double CAR_LENGTH;
    extern double CAR_WIDTH;
    extern double DELTA_THETA; // degree // wheel max turnning angle
    extern double DELTA_FORWARD; // (m) // distance to move forward

    extern int THETA; // degree // this is the zoomed out version of 360 degrees 
    extern int THETA_RES; // degree // Theta*Res = 360
    extern int GX;
    extern int GY;
    extern int GRID_RES;

    // cost map params
    extern double DX; // Astar cost_map resolution
    extern double DY;

    extern double OBS_DIST_MIN; //(m) // minimum dist to nearist obstacle
    extern double DELTA_CHECK; //(m) // delta for distance between points 

}


#endif
