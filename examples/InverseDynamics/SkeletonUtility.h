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
	btQuaternion skeletonNodeRotation(const SkeletonNode &node);
	btMultiBody* createMultiBodyFromSkeletonNodes(const std::vector<SkeletonNode> &skeletonNodes);
	void createMultiBodyColliders(btMultiBodyDynamicsWorld *world, btMultiBody *multiBody);
};

#endif  //SKELETON_UTILITY_H
