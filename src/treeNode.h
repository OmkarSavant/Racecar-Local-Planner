#pragma once

#include <geometry_msgs/Point.h>

struct treeNode {
	geometry_msgs::Point  *node;
	treeNode  *parent;
};

