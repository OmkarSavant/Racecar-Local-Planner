#pragma once

#include "treeNode.h"
#include <unordered_map>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class RRT {
private:
	vector<vector<int>> occu_grid;
	vector<geometry_msgs::Point> nodes;

	const int MAX_DIST; // The max distance between two nodes connected to each other

	ros::NodeHandle n_ros;
	ros::Publisher pub_marker;
	visualization_msgs::Marker points, line_strip, line_list;

	float dist(geometry_msgs::Point& p1, geometry_msgs::Point& p2); 

	geometry_msgs::Point steer(geometry_msgs::Point& p1, geometry_msgs::Point& p2);

	bool _InFreeSpace(geometry_msgs::Point& p);

	bool collision_free(geometry_msgs::Point& p1, geometry_msgs::Point& p2);

	geometry_msgs::Point random_point();

	geometry_msgs::Point findNearest(geometry_msgs::Point &p);

public:
	
	vector<treeNode*> structVect; //vector of nodes structs that are added to the tree
	RRT(vector<vector<int>> occugrid);
	void plan_it(geometry_msgs::Point &p_start, geometry_msgs::Point &p_end, vector<vector<int>> &traj);
	void publish_it();
};
