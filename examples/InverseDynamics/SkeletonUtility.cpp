#include "SkeletonUtility.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

namespace SkeletonUtility
{
	static const btScalar Skeleton_Base_Fixed_Size = 0.4;
	static const btScalar Skeleton_Fixed_Size = 0.4;
	static const int Root_X_Rotation_Degree = -90;
	static const bool Inverse_Rotation_When_Build = true; // todo world to local 顺序

	void calcBoxShapeInertia(const btVector3 &halfExtents, btScalar mass, btVector3 &inertia)
	{
		btBoxShape shape = btBoxShape(halfExtents);
		shape.calculateLocalInertia(mass, inertia);
	}

	btScalar degreeToRad(const btScalar &degree)
	{
		return degree * SIMD_PI / 180;
	}

	btQuaternion skeletonNodeRotation(const SkeletonNode &node, const AnimationKeyTime time /* = AnimationUtility::Invalid_Time */)
	{
		btScalar x = 0, y = 0, z = 0;
		btQuaternion rotation;
		node.getRotaionAtTime(time, x, y, z);
		rotation.setEulerZYX(degreeToRad(z), degreeToRad(y), degreeToRad(x));
		
		btQuaternion rotationPre;
		rotationPre.setEulerZYX(degreeToRad(node.rotationPre[2]), degreeToRad(node.rotationPre[1]), degreeToRad(node.rotationPre[0]));
		rotation = rotation * rotationPre;
		if (node.parentIdx < 0)
		{
			// fbx文件里没找到这90度在哪
			rotation = rotation * btQuaternion(btVector3(1, 0, 0), degreeToRad(Root_X_Rotation_Degree));
		}
		return rotation;
	}

	bool calcNodeWorldToLocalRotations(const std::vector<SkeletonNode> &skeletonNodes, std::vector<btQuaternion> &nodeWorldToLocalRotations, const AnimationKeyTime time /* = AnimationUtility::Invalid_Time */)
	{
		int count = (int)skeletonNodes.size();
		for (int i = 0; i < count; i++)
		{
			const SkeletonNode *node = &skeletonNodes[i];
			btQuaternion worldToLocal = skeletonNodeRotation(*node, time);
			while (node->parentIdx >= 0)
			{
				const SkeletonNode *parentNode = &skeletonNodes[node->parentIdx];
				worldToLocal = skeletonNodeRotation(*parentNode, time) * worldToLocal;
				node = parentNode;
			}
			nodeWorldToLocalRotations.push_back(worldToLocal);
		}
		return true;
	}

	bool calcTransformInfo(const std::vector<SkeletonNode> &skeletonNodes, std::vector<btQuaternion> &nodeWorldToLocalRotations, std::vector<btQuaternion> &jointFrameRotations, const AnimationKeyTime time /* = AnimationUtility::Invalid_Time*/)
	{
		calcNodeWorldToLocalRotations(skeletonNodes, nodeWorldToLocalRotations, time);

		std::vector<btQuaternion> multiBodyWorldToLocalRotations;
		int count = (int)skeletonNodes.size();
		for (int i = 0; i < count; i++)
		{
			const SkeletonNode &node = skeletonNodes[i];
			SkeletonNode::AnimValueType x = 0, y = 0, z = 0;
			node.getTranslationAtTime(time, x, y, z);
			btVector3 toTranslation = btVector3(x, y, z);
			btVector3 fromTranslation = btVector3(toTranslation.length(), 0, 0);
			btScalar angle = fromTranslation.angle(toTranslation);
			btVector3 axis = fromTranslation.cross(toTranslation);
			btQuaternion adjustXAxisRotation = btQuaternion(0, 0, 0, 1);
			if (axis.length() > 0.0)
			{
				adjustXAxisRotation = btQuaternion(axis, angle);
			}

			if (node.parentIdx >= 1)
			{
				// 有关节的骨骼开始偏移
				multiBodyWorldToLocalRotations.push_back(nodeWorldToLocalRotations[node.parentIdx] * adjustXAxisRotation);
			}
			else
			{
				multiBodyWorldToLocalRotations.push_back(nodeWorldToLocalRotations[node.idx]);
			}

		}

		for (int i = 0; i < count; i++)
		{
			const SkeletonNode &node = skeletonNodes[i];
			if (node.parentIdx >= 1)
			{
				btQuaternion jointFrameRotation = multiBodyWorldToLocalRotations[node.parentIdx].inverse() * multiBodyWorldToLocalRotations[node.idx];
				jointFrameRotations.push_back(jointFrameRotation);
			}
			else
			{
				// ??
				jointFrameRotations.push_back(btQuaternion(0, 0, 0, 1));
			}
		}
		return true;
	}

	bool calcJointRotationsAtTime(const std::vector<SkeletonNode> &skeletonNodes, const AnimationKeyTime &time, const std::vector<btQuaternion> &zeroJointRotations, std::vector<btQuaternion> &jointRotations)
	{
		std::vector<btQuaternion> zero, cur;
		calcNodeWorldToLocalRotations(skeletonNodes, zero);
		calcNodeWorldToLocalRotations(skeletonNodes, cur, time);

		int count = (int)skeletonNodes.size();
		for (int i = 0; i < count; i++)
		{
			const SkeletonNode &node = skeletonNodes[i];
			if (node.parentIdx > 0)
			{
				const SkeletonNode &parentNode = skeletonNodes[node.parentIdx];
				btQuaternion baseRotation = skeletonNodeRotation(parentNode);
				btQuaternion curRotation = skeletonNodeRotation(parentNode, time);
				//baseRotation = zero[parentNode.idx];
				//curRotation = cur[parentNode.idx];
				const btQuaternion &curZeroJointRotaion = zeroJointRotations[i];
				btQuaternion rotation = curZeroJointRotaion.inverse() * baseRotation.inverse() * curRotation * curZeroJointRotaion;
				jointRotations.push_back(rotation);
			}
			else
			{
				// ?
				jointRotations.push_back(btQuaternion(0, 0, 0, 1));
			}
		}
		return true;
	}

	btMultiBody *createMultiBodyFromSkeletonNodes(const std::vector<SkeletonNode> &skeletonNodes, 
		std::vector<btQuaternion> &nodeWorldToLocalRotations, std::vector<btQuaternion> &jointFrameRotations)
	{
		int skeletonNodesCount = skeletonNodes.size();
		if (skeletonNodesCount < 2)
		{
			return NULL;
		}

		calcTransformInfo(skeletonNodes, nodeWorldToLocalRotations, jointFrameRotations);

		int numLinks = skeletonNodesCount - 2;
		btScalar mass = 1.0;
		btVector3 baseInertia; // todo，这是算的对角线转动惯量？？
		calcBoxShapeInertia(btVector3(Skeleton_Base_Fixed_Size / 2, Skeleton_Base_Fixed_Size / 2, Skeleton_Base_Fixed_Size / 2), mass, baseInertia);

		bool fixedBase = true;
		bool canSleep = false; // todo， what mean
		btMultiBody* multiBody = new btMultiBody(numLinks, mass, baseInertia, fixedBase, canSleep);
		const SkeletonNode &rootNode = skeletonNodes[0];
		btVector3 pos = btVector3(rootNode.translation[0], rootNode.translation[1], rootNode.translation[2]);

		btQuaternion rotation = nodeWorldToLocalRotations[1];  // todo 这里用pelvis的rotation?

		multiBody->setBasePos(pos);
		multiBody->setWorldToBaseRot(rotation);
		//btTransform baseTransform = btTransform(rotation.inverse(), pos);
		//multiBody->setBaseWorldTransform(baseTransform);

		bool disableParentCollision = true; // todo， what mean

		btVector3 hingeJointAxis(1, 0, 0);
		int startIdxOffset = 2;
		btScalar skeletonFixedSize = Skeleton_Fixed_Size;
		for (int i = startIdxOffset; i < skeletonNodesCount; ++i)
		{
			const SkeletonNode &curNode = skeletonNodes[i];
			const SkeletonNode &parentNode = skeletonNodes[curNode.parentIdx];
			btVector3 curTranslation = btVector3(curNode.translation[0], curNode.translation[1], curNode.translation[2]);
			btVector3 parentTranslation = btVector3(parentNode.translation[0], parentNode.translation[1], parentNode.translation[2]);
			btScalar skeletonLength = curTranslation.length();
			btScalar parentSkeletonLength = parentTranslation.length();
			btVector3 inertiaDiag;
			calcBoxShapeInertia(btVector3(skeletonLength / 2, skeletonFixedSize / 2, skeletonFixedSize / 2), mass, inertiaDiag);
			btQuaternion jointFrameRotation = jointFrameRotations[i];
			if (Inverse_Rotation_When_Build)
			{
				// todo world to local 顺序
				jointFrameRotation = jointFrameRotation.inverse();
			}
			multiBody->setupSpherical(i - startIdxOffset, mass, inertiaDiag, parentNode.idx - startIdxOffset, jointFrameRotation,
				btVector3(parentSkeletonLength / 2, 0, 0), btVector3(skeletonLength / 2, 0, 0), disableParentCollision);

			/*
			multiBody->setupRevolute(i - startIdxOffset, mass, inertiaDiag, parentNode.idx - startIdxOffset, 
				jointFrameRotations[i], hingeJointAxis, btVector3(parentSkeletonLength / 2, 0, 0), btVector3(skeletonLength / 2, 0, 0), disableParentCollision);
				*/
		}

		multiBody->finalizeMultiDof();

		multiBody->setHasSelfCollision(true);

		multiBody->setLinearDamping(0.1f);
		multiBody->setAngularDamping(0.1f);

		return multiBody;
	}

	void createMultiBodyColliders(btMultiBodyDynamicsWorld *world, btMultiBody *multiBody)
	{
		const btVector3 &basePos = multiBody->getBasePos();
		const btQuaternion &baseRotation = multiBody->getWorldToBaseRot();
		btBoxShape *baseBox = new btBoxShape(btVector3(Skeleton_Base_Fixed_Size / 2, Skeleton_Base_Fixed_Size / 2, Skeleton_Base_Fixed_Size / 2));
		btMultiBodyLinkCollider* baseCollider = new btMultiBodyLinkCollider(multiBody, -1);
		// todo shape、 collider的内存??
		baseCollider->setCollisionShape(baseBox);
		// todo friction ??
		float friction = 1;
		baseCollider->setFriction(friction);
		// todo demo这里rotate 角度用的负的？

		btTransform baseTransform = multiBody->getBaseWorldTransform();
		baseCollider->setWorldTransform(baseTransform);

		std::vector<btTransform> transforms;
		transforms.push_back(baseTransform);

		multiBody->setBaseCollider(baseCollider);

		world->addCollisionObject(baseCollider);

		btVector3 curPos = basePos;
		int numLinks = multiBody->getNumLinks();
		btScalar skeletonFixedSize = Skeleton_Fixed_Size;


		for (int i = 0; i < numLinks; ++i)
		{
			// todo shape、 collider的内存??
			btVector3 halfSkeletonVector = multiBody->getLink(i).m_dVector;
			btScalar halfSkeletonLength = halfSkeletonVector.length();
			btBoxShape *box = new btBoxShape(btVector3(halfSkeletonLength, skeletonFixedSize / 2, skeletonFixedSize / 2));
			btMultiBodyLinkCollider* collider = new btMultiBodyLinkCollider(multiBody, i);
			collider->setCollisionShape(box);
			collider->setFriction(friction);

			btQuaternion rotation = multiBody->getLink(i).m_zeroRotParentToThis;
			if (Inverse_Rotation_When_Build)
			{
				// todo world to local 顺序
				rotation = rotation.inverse();
			}
			btTransform curRotationTransform = btTransform(rotation);
			btTransform curCOMTransform = curRotationTransform * btTransform(btQuaternion(0, 0, 0, 1), btVector3(halfSkeletonLength, 0, 0));
			btTransform curEndTransform = curRotationTransform * btTransform(btQuaternion(0, 0, 0, 1), btVector3(halfSkeletonLength * 2, 0, 0));

			btTransform curCOMWorldTransform = transforms[multiBody->getParent(i) + 1] * curCOMTransform;
			btTransform curEndWorldTransform = transforms[multiBody->getParent(i) + 1] * curEndTransform;
			collider->setWorldTransform(curCOMWorldTransform);
			transforms.push_back(curEndWorldTransform);

			multiBody->getLink(i).m_collider = collider;

			world->addCollisionObject(collider);
		}
	}
}

