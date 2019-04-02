#ifndef SKELETON_UTILITY_H
#define SKELETON_UTILITY_H

#include <vector>
#include "SkeletonNode.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"

class btMultiBody;
class btMultiBodyDynamicsWorld;

namespace SkeletonUtility
{
	void calcBoxShapeInertia(const btVector3 &halfExtents, btScalar mass, btVector3 &inertia);

	btScalar degreeToRad(const btScalar &degree);

	btQuaternion calcRotationAtTime(const SkeletonNode &node, const AnimationKeyTime time = AnimationUtility::Invalid_Time);

	btVector3 calcTranslationAtTime(const SkeletonNode &node, const AnimationKeyTime time = AnimationUtility::Invalid_Time);

	btTransform calcBaseTransformAtTime(const std::vector<SkeletonNode> &skeletonNodes, const AnimationKeyTime time = AnimationUtility::Invalid_Time);

	bool calcNodeWorldToLocalRotationsAtTime(const std::vector<SkeletonNode> &skeletonNodes, 
		std::vector<btQuaternion> &nodeWorldToLocalRotations, const AnimationKeyTime time = AnimationUtility::Invalid_Time);

	bool calcTransformInfoAtTime(const std::vector<SkeletonNode> &skeletonNodes, std::vector<btQuaternion> &nodeWorldToLocalRotations, 
		std::vector<btQuaternion> &jointFrameRotations, const AnimationKeyTime time = AnimationUtility::Invalid_Time);

	bool calcJointDofAtTime(const std::vector<SkeletonNode> &skeletonNodes, const std::vector<btQuaternion> &zeroJointRotations,
		std::vector<btQuaternion> &jointDOFs, const AnimationKeyTime time = AnimationUtility::Invalid_Time);

	btMultiBody *createMultiBodyFromSkeletonNodes(const std::vector<SkeletonNode> &skeletonNodes, const AnimationKeyTime &startTime, std::vector<btQuaternion> &jointFrameRotations);

	void createMultiBodyColliders(btMultiBodyDynamicsWorld *world, btMultiBody *multiBody);

	void calcJointStates(const double deltaTime, const std::vector<btQuaternion> &startQ, const std::vector<btVector3> &startDotQ, 
		const std::vector<btQuaternion> &endQ, std::vector<btVector3> &endDotQ, std::vector<btVector3> &dotDotQ);
};

#endif  //SKELETON_UTILITY_H
