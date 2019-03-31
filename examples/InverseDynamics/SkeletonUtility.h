#ifndef SKELETON_UTILITY_H
#define SKELETON_UTILITY_H

#include <vector>
#include "SkeletonNode.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

class btMultiBody;
class btMultiBodyDynamicsWorld;

namespace SkeletonUtility
{
	void calcBoxShapeInertia(const btVector3 &halfExtents, btScalar mass, btVector3 &inertia);

	btScalar degreeToRad(const btScalar &degree);

	btQuaternion skeletonNodeRotation(const SkeletonNode &node, const AnimationKeyTime time = AnimationUtility::Invalid_Time);

	bool calcNodeWorldToLocalRotations(const std::vector<SkeletonNode> &skeletonNodes, 
		std::vector<btQuaternion> &nodeWorldToLocalRotations, const AnimationKeyTime time = AnimationUtility::Invalid_Time);

	bool calcTransformInfo(const std::vector<SkeletonNode> &skeletonNodes,
						   std::vector<btQuaternion> &nodeWorldToLocalRotations,
						   std::vector<btQuaternion> &jointFrameRotations,
						   const AnimationKeyTime time = AnimationUtility::Invalid_Time);

	bool calcJointRotationsAtTime(const std::vector<SkeletonNode> &skeletonNodes, const AnimationKeyTime &time, const std::vector<btQuaternion> &zeroJointRotations, std::vector<btQuaternion> &jointRotations);

	btMultiBody *createMultiBodyFromSkeletonNodes(const std::vector<SkeletonNode> &skeletonNodes, 
		std::vector<btQuaternion> &nodeWorldToLocalRotations, std::vector<btQuaternion> &jointFrameRotations);

	void createMultiBodyColliders(btMultiBodyDynamicsWorld *world, btMultiBody *multiBody);
};

#endif  //SKELETON_UTILITY_H
