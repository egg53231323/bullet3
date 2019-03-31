#include "SkeletonNode.h"

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

void SkeletonNode::getTranslationAtTime(const AnimationKeyTime &time, AnimValueType &x, AnimValueType &y, AnimValueType &z) const
{
	if (time == AnimationUtility::Invalid_Time)
	{
		x = translation[0];
		y = translation[1];
		z = translation[2];
		return;
	}

	x = animationT[0].getValue(time, translation[0]);
	y = animationT[1].getValue(time, translation[1]);
	z = animationT[2].getValue(time, translation[2]);
}

void SkeletonNode::getRotaionAtTime(const AnimationKeyTime &time, AnimValueType &x, AnimValueType &y, AnimValueType &z) const
{
	if (time == AnimationUtility::Invalid_Time)
	{
		x = rotation[0];
		y = rotation[1];
		z = rotation[2];
		return;
	}

	x = animationR[0].getValue(time, rotation[0]);
	y = animationR[1].getValue(time, rotation[1]);
	z = animationR[2].getValue(time, rotation[2]);
}

void SkeletonNode::getScaleAtTime(const AnimationKeyTime &time, AnimValueType &x, AnimValueType &y, AnimValueType &z) const
{
	if (time == AnimationUtility::Invalid_Time)
	{
		x = scale[0];
		y = scale[1];
		z = scale[2];
		return;
	}

	x = animationS[0].getValue(time, scale[0]);
	y = animationS[1].getValue(time, scale[1]);
	z = animationS[2].getValue(time, scale[2]);
}