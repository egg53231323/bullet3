#include "SkeletonNode.h"

SkeletonAnimationKey::SkeletonAnimationKey() : time(0), value(0)
{

}

SkeletonNode::SkeletonNode() : idx(0), parentIdx(0)
{
	translation[0] = translation[1] = translation[2] = 0;
	rotation[0] = rotation[1] = rotation[2] = 0;
	scale[0] = scale[1] = scale[2] = 1;

	rotationPre[0] = rotationPre[1] = rotationPre[2] = 0;
}

void SkeletonNode::setTranslation(const double &x, const double &y, const double &z)
{
	translation[0] = x;
	translation[1] = y;
	translation[2] = z;
}

void SkeletonNode::setRotation(const double &x, const double &y, const double &z)
{
	rotation[0] = x;
	rotation[1] = y;
	rotation[2] = z;
}

void SkeletonNode::setScale(const double &x, const double &y, const double &z)
{
	scale[0] = x;
	scale[1] = y;
	scale[2] = z;
}

void SkeletonNode::setRotationPre(const double &x, const double &y, const double &z)
{
	rotationPre[0] = x;
	rotationPre[1] = y;
	rotationPre[2] = z;
}

