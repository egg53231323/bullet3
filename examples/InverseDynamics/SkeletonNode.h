#ifndef SKELETON_NODE_H
#define SKELETON_NODE_H

#include <string>

class SkeletonNode
{
public:
	SkeletonNode();
	std::string name;
	int idx;
	int parentIdx;
	double translation[3];
	double rotation[3];
	double scale[3];

	double rotationPre[3];
};

#endif  //SKELETON_NODE_H
