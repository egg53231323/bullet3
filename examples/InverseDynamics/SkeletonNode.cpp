#include "SkeletonNode.h"

SkeletonNode::SkeletonNode() : idx(0), parentIdx(0)
{
	translation[0] = translation[1] = translation[2] = 0;
	rotation[0] = rotation[1] = rotation[2] = 0;
	scale[0] = scale[1] = scale[2] = 1;

	rotationPre[0] = rotationPre[1] = rotationPre[2] = 0;
}
