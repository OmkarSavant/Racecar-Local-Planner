#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


using namespace std;

class RRT_star{
private:	

	vector<vector<int>> occu_grid;
	vector<geometry_msgs::Point> nodes;
	unordered_map<geometry_msgs::Point,vector<geometry_msgs::Point>> adj_tree;
	int x_max = 100;
	int y_max = 100;
	const int MAX_DIST = 10;


	int dist(geometry_msgs::Point& p1, geometry_msgs::Point& p2){
		// returning int might result in bug!!!
		return sqrt(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
	}

	geometry_msgs::Point steer(geometry_msgs::Point& p1, geometry_msgs::Point& p2){
		if (dist(p1,p2) <= MAX_DIST) return p2;
		else {
			float the = atan2((p2.y - p1.y),(p2.x - p1.x));
			geometry_msgs::Point temp_p;
			temp_p.x = p1.x + MAX_DIST * cos(the);
			temp_p.y = p1.y + MAX_DIST * sin(the);
			return temp_p;
		}
	}

	// TODO!!!
	bool collision_free(geometry_msgs::Point& p1, geometry_msgs::Point& p2){
		// check if the path between p1 and p2 is collision free or not
		float slope = (p2.y - p1.y)/(p2.x - p1.x);
		float c = p2.y - p2.x * slope;
		return true;
	}

	bool _InFreeSpace(geometry_msgs::Point &p){
		if (occu_grid[p.x][p.y]==0) return true;
		else return false;
	}

	geometry_msgs::Point random_point(const int x_max, const int y_max){
		geometry_msgs::Point temp_p;
		temp_p.x = rand() % x_max + 1;
		temp_p.y = rand() % y_max + 1;
		temp_p.z = 0;
		return temp_p;
	}

	geometry_msgs::Point findNearest(geometry_msgs::Point &p){
		geometry_msgs::Point n = nodes[0];
		for (auto node:nodes){
			if (dist(p,node)<dist(p,n)){
				n = node;
			}
		}
		return n;
	}



public:
	RRT_star(){
		cout<<"The RRT_star object in being called"<<endl;
		ros::init(	argc, argv, "rrt_points_lines");
		ros::NodeHandle n_ros;
		ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>("viz_marker",10);
		
	}

	void plan_it(geometry_msgs::Point p_start, geometry_msgs::Point p_end){

		// init ??
		nodes.push_back(p_start);
	    const int N = 10000;
	    for (int i=0; i<N; i++){
	    	geometry_msgs::Point rd = random_point();
	    	while (!_InFreeSpace(rd)) rd = random_point();
	    	geometry_msgs::Point n = findNearest(rd);
	    	rd = steer(n,rd);
	    	if (collision_free(n,rd)){
	    		// connect...
	    		// DO SOMETHING!!!!!
	    	}
	    	//visulize
	    }
	}


}