#ifndef SKELETON_NODE_H
#define SKELETON_NODE_H

#include <string>
#include <vector>
#include "AnimationUtility.h"

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

	typedef double AnimValueType;
	AnimationCurve<AnimValueType> animationT[3];
	AnimationCurve<AnimValueType> animationR[3];
	AnimationCurve<AnimValueType> animationS[3];

	void setTranslation(const double &x, const double &y, const double &z);
	void setRotation(const double &x, const double &y, const double &z);
	void setScale(const double &x, const double &y, const double &z);
	void setRotationPre(const double &x, const double &y, const double &z);

	void getTranslationAtTime(const AnimationKeyTime &time, AnimValueType &x, AnimValueType &y, AnimValueType &z) const;
	void getRotaionAtTime(const AnimationKeyTime &time, AnimValueType &x, AnimValueType &y, AnimValueType &z) const;
	void getScaleAtTime(const AnimationKeyTime &time, AnimValueType &x, AnimValueType &y, AnimValueType &z) const;
};

#endif  //SKELETON_NODE_H
