#include "SkeletonUtility.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

namespace SkeletonUtility
{
	static const btScalar Skeleton_Base_Fixed_Size = 0.4;
	static const btScalar Skeleton_Fixed_Size = 0.4;
	static const int Root_X_Rotation_Degree = -90;
	static const bool Inverse_Rotation_When_Build = true; // todo world to local ˳��

	JointAnimationKey::JointAnimationKey() : time(0), rotation(0, 0, 0, 1)
	{

	}

	void calcBoxShapeInertia(const btVector3 &halfExtents, btScalar mass, btVector3 &inertia)
	{
		btBoxShape shape = btBoxShape(halfExtents);
		shape.calculateLocalInertia(mass, inertia);
	}

	btScalar degreeToRad(const btScalar &degree)
	{
		return degree * SIMD_PI / 180;
	}

	btQuaternion skeletonNodeRotation(const SkeletonNode &node)
	{
		btQuaternion rotation;
		rotation.setEulerZYX(degreeToRad(node.rotation[2]), degreeToRad(node.rotation[1]), degreeToRad(node.rotation[0]));
		btQuaternion rotationPre;
		rotationPre.setEulerZYX(degreeToRad(node.rotationPre[2]), degreeToRad(node.rotationPre[1]), degreeToRad(node.rotationPre[0]));
		rotation = rotation * rotationPre;
		if (node.parentIdx < 0)
		{
			// fbx�ļ���û�ҵ���90������
			rotation = rotation * btQuaternion(btVector3(1, 0, 0), degreeToRad(Root_X_Rotation_Degree));
		}
		return rotation;
	}

	bool calcTransAnimation(const std::vector<SkeletonNode> &skeletonNodes, const std::vector<btQuaternion> &srcModelWorldToLocalRotations, 
		const std::vector<btQuaternion> &multiBodyWorldToLocalRotations, const std::vector<btQuaternion> &jointFrameRotations, 
		std::vector<std::vector<JointAnimationKey> > &jointRotationAnimation)
	{
		int nodeCount = (int)skeletonNodes.size();
		for (int i = 0; i < nodeCount; i++)
		{
			const SkeletonNode &skeletonNode = skeletonNodes[i];
		}

		return true;
	}

	bool calcTransformInfo(const std::vector<SkeletonNode> &skeletonNodes, std::vector<btQuaternion> &srcModelWorldToLocalRotations, std::vector<btQuaternion> &jointFrameRotations)
	{
		std::vector<btQuaternion> multiBodyWorldToLocalRotations;
		int count = (int)skeletonNodes.size();
		for (int i = 0; i < count; i++)
		{
			const SkeletonNode *node = &skeletonNodes[i];
			btQuaternion worldToLocal = skeletonNodeRotation(*node);
			while (node->parentIdx >= 0)
			{
				const SkeletonNode *parentNode = &skeletonNodes[node->parentIdx];
				worldToLocal = skeletonNodeRotation(*parentNode) * worldToLocal;
				node = parentNode;
			}
			srcModelWorldToLocalRotations.push_back(worldToLocal);
		}

		for (int i = 0; i < count; i++)
		{
			const SkeletonNode &node = skeletonNodes[i];
			btVector3 toTranslation = btVector3(node.translation[0], node.translation[1], node.translation[2]);
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
				// �йؽڵĹ�����ʼƫ��
				multiBodyWorldToLocalRotations.push_back(srcModelWorldToLocalRotations[node.parentIdx] * adjustXAxisRotation);
			}
			else
			{
				multiBodyWorldToLocalRotations.push_back(srcModelWorldToLocalRotations[node.idx]);
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

	btMultiBody* createMultiBodyFromSkeletonNodes(const std::vector<SkeletonNode> &skeletonNodes)
	{
		int skeletonNodesCount = skeletonNodes.size();
		if (skeletonNodesCount < 2)
		{
			return NULL;
		}

		std::vector<btQuaternion> worldToLocalRotations, jointFrameRotations;
		calcTransformInfo(skeletonNodes, worldToLocalRotations, jointFrameRotations);

		int numLinks = skeletonNodesCount - 2;
		btScalar mass = 1.0;
		btVector3 baseInertia; // todo��������ĶԽ���ת����������
		calcBoxShapeInertia(btVector3(Skeleton_Base_Fixed_Size / 2, Skeleton_Base_Fixed_Size / 2, Skeleton_Base_Fixed_Size / 2), mass, baseInertia);

		bool fixedBase = true;
		bool canSleep = false; // todo�� what mean
		btMultiBody* multiBody = new btMultiBody(numLinks, mass, baseInertia, fixedBase, canSleep);
		const SkeletonNode &rootNode = skeletonNodes[0];
		btVector3 pos = btVector3(rootNode.translation[0], rootNode.translation[1], rootNode.translation[2]);

		btQuaternion rotation = worldToLocalRotations[1]; // todo ������pelvis��rotation?

		multiBody->setBasePos(pos);
		multiBody->setWorldToBaseRot(rotation);
		//btTransform baseTransform = btTransform(rotation.inverse(), pos);
		//multiBody->setBaseWorldTransform(baseTransform);

		bool disableParentCollision = true; // todo�� what mean

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
				// todo world to local ˳��
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
		// todo shape�� collider���ڴ�??
		baseCollider->setCollisionShape(baseBox);
		// todo friction ??
		float friction = 1;
		baseCollider->setFriction(friction);
		// todo demo����rotate �Ƕ��õĸ��ģ�

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
			// todo shape�� collider���ڴ�??
			btVector3 halfSkeletonVector = multiBody->getLink(i).m_dVector;
			btScalar halfSkeletonLength = halfSkeletonVector.length();
			btBoxShape *box = new btBoxShape(btVector3(halfSkeletonLength, skeletonFixedSize / 2, skeletonFixedSize / 2));
			btMultiBodyLinkCollider* collider = new btMultiBodyLinkCollider(multiBody, i);
			collider->setCollisionShape(box);
			collider->setFriction(friction);

			btQuaternion rotation = multiBody->getLink(i).m_zeroRotParentToThis;
			if (Inverse_Rotation_When_Build)
			{
				// todo world to local ˳��
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
