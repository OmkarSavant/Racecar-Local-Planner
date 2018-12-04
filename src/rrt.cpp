#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <unordered_map>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <chrono>

using namespace std;

class RRT{
private:	
	int x_max = 100; // Max N of rows; Could be initiated at the begining 
	int y_max = 100; // Max N of cols;
	vector<vector<int>> occu_grid {x_max, vector<int>(y_max,0)}; //init the grid with x_max and y_max
	vector<geometry_msgs::Point> nodes; // List of all points from rrt
	// unordered_map<geometry_msgs::Point,vector<geometry_msgs::Point>> adj_tree; // Build the adjancent tree

	const int MAX_DIST = 5; // The max distance between two nodes connected to each other

	// ros::init(argc, argv, "rrt_points_lines");
	ros::NodeHandle n_ros;
	ros::Publisher pub_marker = n_ros.advertise<visualization_msgs::Marker>("viz_marker",10);

    visualization_msgs::Marker points, line_strip, line_list;

	// Calculates the distance between two nodes
	float dist(geometry_msgs::Point& p1, geometry_msgs::Point& p2){
		// returning int might result in bug!!!
		return sqrt(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
	}

	// Steer random node towards the closet node
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

	// Find if the path between two points is collision free
	bool collision_free(geometry_msgs::Point& p1, geometry_msgs::Point& p2){
		// check if the path between p1 and p2 is collision free or not
		float slope = 0.0;
		float c = 0.0;

		if ((p2.y-p1.y)==0 && (p2.x-p1.x)==0) return false;

		else if ((p2.y-p1.y)==0){
			int x_small = min(p1.x,p2.x);
			int x_large = max(p1.x,p2.x);
			for (int i=x_small; i<=x_large; i++){
				if (occu_grid[i][p1.y]==1) return false;
			}
			return true;
		}

		else if ((p2.x-p1.x)==0){
			int y_small = min(p2.y,p1.y);
			int y_large = max(p2.y,p1.y);
			for (int i=y_small; i<=y_large; i++){
				if (occu_grid[p1.x][i]==1) return false;
			}
			return true;
		}
		else {
			slope = (p2.y - p1.y)/(p2.x - p1.x);
			c = p1.y - p1.x * slope;
			for (float i=p1.x; i<p2.x;i+=0.1){
				int y_cal = slope*i+c;
				int x_cal = static_cast<int>(i);
				if (occu_grid[x_cal][y_cal]==1) return false;
			}
			return true;			
		}

	}

	// Check if the random node is in free space
	bool _InFreeSpace(geometry_msgs::Point& p){
		if (occu_grid[p.x][p.y]==0) return true;
		else return false;
	}

	// Generate a random point
	geometry_msgs::Point random_point(){
		geometry_msgs::Point temp_p;
		temp_p.x = rand() % x_max ;
		temp_p.y = rand() % y_max ;
		temp_p.z = 0;
		return temp_p;
	}

	// Find the nearest point from the random point
	geometry_msgs::Point findNearest(geometry_msgs::Point &p){
		geometry_msgs::Point n = nodes[0];
		for (auto node:nodes){
			// cout<<"node"<<node.x<<" "<<node.y<<endl;
			// cout<<"n[0]"<<n.x<<" "<<n.y<<endl;
			// cout<<"p"<<p.x<<" "<<p.y<<endl;
			if (dist(p,node)<dist(p,n)){
				n = node;
			}
		}
		return n;
	}



public:
	RRT(){
		cout<<"The RRT_star object in being called"<<endl;		
	    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
	    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
	    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
	    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
	    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

	    points.type = visualization_msgs::Marker::POINTS;

	    points.scale.x = 0.2;
	    points.scale.y = 0.2;
	    points.color.g = 1.0f;
	    points.color.a = 1.0;
        line_list.scale.x = 0.1;
	    // Line list is red
	    line_list.id = 2;
    	line_list.type = visualization_msgs::Marker::LINE_LIST;
	    line_list.color.r = 1.0;
	    line_list.color.a = 1.0;
		cout<<"The RRT_star object in being called agian"<<endl;		
	}

	void plan_it(geometry_msgs::Point &p_start, geometry_msgs::Point &p_end){

		// init ??
		nodes.push_back(p_start);
	    const int N = 1000;
	    auto start = std::chrono::high_resolution_clock::now();

	    for (int i=0; i<N; i++){
	    	// cout<< "here"<<endl;
	    	geometry_msgs::Point rd = random_point();
	    	while (!_InFreeSpace(rd)) rd = random_point();
	    	geometry_msgs::Point n = findNearest(rd);
	    	// cout << n.x<<n.y<<endl;
	    	// rd = steer(n,rd);
	    	if (collision_free(n,rd)){
	    		// adj_tree[n].push_back(rd);
	    		points.points.push_back(rd);
	    		line_list.points.push_back(n);
	    		line_list.points.push_back(rd);
	    		nodes.push_back(rd);

	    		// DO SOMETHING!!!!!
	    	}
	    	pub_marker.publish(points);
	    	pub_marker.publish(line_list);
	    	// cout<< "publised"<<endl;

	    }

	    auto finish = std::chrono::high_resolution_clock::now();
	    std::chrono::duration<double> elapsed = finish - start;
		std::cout << "Elapsed time: " << elapsed.count() << " s\n";
	}

	void publish_it(){
    	pub_marker.publish(points);
    	pub_marker.publish(line_list);
	}
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "rrt_points_lines");	
	RRT rrt;
  	ros::Rate r(5);
    cout << "Press Enter to Continue";
	cin.ignore();
	geometry_msgs::Point P1,P2;
	P1.x = 0;
	P1.y = 0;
	P2.x = 10;
	P2.y = 10;
	cout<<"just before plan"<<endl;	
	rrt.plan_it(P1,P2);
		while (ros::ok()){	
			rrt.publish_it();
			r.sleep();}
 //    cout << "Press Enter to Continue";
	// cin.ignore();
	// r.sleep();
// }

	return 0;
}