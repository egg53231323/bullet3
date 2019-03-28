#include "InverseDynamicsTest.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../Utils/b3ResourcePath.h"
#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "BulletInverseDynamics/IDConfig.hpp"
#include "../Extras/InverseDynamics/btMultiBodyTreeCreator.hpp"

#include "../RenderingExamples/TimeSeriesCanvas.h"

#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "../../Utils/b3BulletDefaultFileIO.h"
#include "../Importers//ImportMeshUtility/b3ImportMeshUtility.h"

#include <vector>
#include "FbxUtility.h"

static const btScalar Skeleton_Base_Fixed_Size = 0.4;
static const btScalar Skeleton_Fixed_Size = 0.4;

bool modifyTranslationToX(std::vector<SkeletonNode> &skeletonNodes)
{
	int nodeCount = skeletonNodes.size();
	for (int i = 2; i < nodeCount; i++)
	{
		SkeletonNode &curNode = skeletonNodes[i];
		btVector3 srcTranslation = btVector3(curNode.translation[0], curNode.translation[1], curNode.translation[2]);
		btVector3 dstTranslation = btVector3(srcTranslation.length(), 0, 0);
		btQuaternion quanternionA;
		quanternionA.setEulerZYX(curNode.rotation[2], curNode.rotation[1], curNode.rotation[0]);

		btQuaternion quanternionB = shortestArcQuat(dstTranslation, srcTranslation);
		btQuaternion dstQuanternion = quanternionA * quanternionB;

		curNode.translation[0] = dstTranslation.x();
		curNode.translation[1] = dstTranslation.y();
		curNode.translation[2] = dstTranslation.z();
		dstQuanternion.getEulerZYX(curNode.rotation[2], curNode.rotation[1], curNode.rotation[0]);
	}
	return true;
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
	btQuaternion worldToLocal;
	worldToLocal.setEulerZYX(degreeToRad(node.rotation[2]), degreeToRad(node.rotation[1]), degreeToRad(node.rotation[0]));
	return worldToLocal;
}

bool calcTransformInfo(const std::vector<SkeletonNode> &skeletonNodes, std::vector<btQuaternion> &nodeWorldToLocalRotations, std::vector<btQuaternion> &jointFrameRotations)
{
	std::vector<btQuaternion> worldToLocalRotations;
	std::vector<btQuaternion> nodeAdjustXAxisRotations;
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
		nodeWorldToLocalRotations.push_back(worldToLocal);
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
		
		nodeAdjustXAxisRotations.push_back(adjustXAxisRotation);
		if (node.parentIdx >= 1)
		{
			worldToLocalRotations.push_back(nodeWorldToLocalRotations[node.parentIdx] * adjustXAxisRotation);
		}
		else
		{
			worldToLocalRotations.push_back(btQuaternion(0, 0, 0, 1));
		}
		
	}

	for (int i = 0; i < count; i++)
	{
		const SkeletonNode &node = skeletonNodes[i];
		if (node.parentIdx >= 1)
		{	
			btQuaternion jointFrameRotation = worldToLocalRotations[node.parentIdx].inverse() * worldToLocalRotations[node.idx];
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
	btVector3 baseInertia; // todo，这是算的对角线转动惯量？？
	calcBoxShapeInertia(btVector3(Skeleton_Base_Fixed_Size / 2, Skeleton_Base_Fixed_Size / 2, Skeleton_Base_Fixed_Size / 2), mass, baseInertia);

	bool fixedBase = true;
	bool canSleep = false; // todo， what mean
	btMultiBody* multiBody = new btMultiBody(numLinks, mass, baseInertia, fixedBase, canSleep);
	const SkeletonNode &rootNode = skeletonNodes[0];
	btVector3 pos = btVector3(rootNode.translation[0], rootNode.translation[1], rootNode.translation[2]);

	btQuaternion rotation = worldToLocalRotations[1]; // todo 这里用pelvis的rotation?
	btTransform baseTransform = btTransform(rotation, pos);

	multiBody->setBasePos(pos);
	multiBody->setWorldToBaseRot(rotation);
	// multiBody->setBaseWorldTransform(baseTransform);

	bool disableParentCollision = true; // todo， what mean

	btVector3 hingeJointAxis(1, 0, 0);
	int startIdxOffset = 2;
	btScalar skeletonFixedSize = Skeleton_Fixed_Size;
	for (int i = startIdxOffset; i < skeletonNodesCount; ++i)
	{
		const SkeletonNode &curNode = skeletonNodes[i];
		const SkeletonNode &parentNode = skeletonNodes[curNode.parentIdx];
		//multiBody->setupRevolute(i, mass, inertiaDiag, i - 1, btQuaternion(0, 0, 0, 1), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, disableParentCollision);
		btVector3 curTranslation = btVector3(curNode.translation[0], curNode.translation[1], curNode.translation[2]);
		btVector3 parentTranslation = btVector3(parentNode.translation[0], parentNode.translation[1], parentNode.translation[2]);
		btScalar skeletonLength = curTranslation.length();
		btScalar parentSkeletonLength = parentTranslation.length();
		btVector3 inertiaDiag;
		calcBoxShapeInertia(btVector3(skeletonLength / 2, skeletonFixedSize / 2, skeletonFixedSize / 2), mass, inertiaDiag);
		multiBody->setupSpherical(i - startIdxOffset, mass, inertiaDiag, parentNode.idx - startIdxOffset, jointFrameRotations[i],
			btVector3(parentSkeletonLength / 2, 0, 0), btVector3(skeletonLength / 2, 0, 0), disableParentCollision);
	}

	multiBody->finalizeMultiDof();

	multiBody->setHasSelfCollision(true);

	multiBody->setLinearDamping(0.0f);
	multiBody->setAngularDamping(0.0f);

	return multiBody;
}


void createMultiBodyColliders(btMultiBodyDynamicsWorld *world, btMultiBody *multiBody)
{
	const btQuaternion &baseRotation = multiBody->getWorldToBaseRot();
	const btVector3 &basePos = multiBody->getBasePos();
	btBoxShape *baseBox = new btBoxShape(btVector3(Skeleton_Base_Fixed_Size / 2, Skeleton_Base_Fixed_Size / 2, Skeleton_Base_Fixed_Size / 2));
	btMultiBodyLinkCollider* baseCollider = new btMultiBodyLinkCollider(multiBody, -1);
	// todo shape、 collider的内存??
	baseCollider->setCollisionShape(baseBox);
	// todo friction ??
	float friction = 1;
	baseCollider->setFriction(friction);
	// todo demo这里rotate 角度用的负的？

	btTransform baseTransform = btTransform(baseRotation, basePos);
	baseCollider->setWorldTransform(baseTransform);

	multiBody->setBaseCollider(baseCollider);

	world->addCollisionObject(baseCollider);

	btQuaternion curRotation = baseRotation;
	btVector3 curPos = basePos;
	int numLinks = multiBody->getNumLinks();
	btScalar skeletonFixedSize = Skeleton_Fixed_Size;
	
	std::vector<btTransform> transforms;
	transforms.push_back(baseTransform);
	for (int i = 0; i < numLinks; ++i)
	{
		// todo shape、 collider的内存??
		btVector3 halfSkeletonVector = multiBody->getLink(i).m_dVector;
		btScalar halfSkeletonLength = halfSkeletonVector.length();
		btBoxShape *box = new btBoxShape(btVector3(halfSkeletonLength, skeletonFixedSize / 2, skeletonFixedSize / 2));
		btMultiBodyLinkCollider* collider = new btMultiBodyLinkCollider(multiBody, i);
		collider->setCollisionShape(box);
		collider->setFriction(friction);

		btTransform curRotationTransform = btTransform(multiBody->getParentToLocalRot(i));
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


class InverseDynamicsTest : public CommonMultiBodyBase
{
	btMultiBody* m_multiBody;
	btInverseDynamics::MultiBodyTree* m_inverseModel;

public:
	InverseDynamicsTest(struct GUIHelperInterface* helper);
	virtual ~InverseDynamicsTest();

	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);
	btMultiBody* createTestMultiBody(const btVector3 &boneHalfExtents);
	void createMultiBodyColliders(btMultiBodyDynamicsWorld *world, btMultiBody *body, const btVector3 &boneHalfExtents);
	btInverseDynamics::MultiBodyTree* createInverseDynamicModel(btMultiBody *body);
	void calcForceWithKenimaticData(btInverseDynamics::MultiBodyTree* inverseModel);

	virtual void resetCamera()
	{
		float dist = 1;
		float pitch = -20;
		float yaw = 180;
		float targetPos[3] = { 0, 30, 6 };
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

InverseDynamicsTest::InverseDynamicsTest(struct GUIHelperInterface* helper)
	: CommonMultiBodyBase(helper),
	m_multiBody(0)
{
}

InverseDynamicsTest::~InverseDynamicsTest()
{

}

void InverseDynamicsTest::initPhysics()
{
	bool useFBX = true;

	std::vector<SkeletonNode> skeletonNodes;
	FbxUtility::loadFbxFile("E:\\work\\motion\\nobody.FBX", skeletonNodes);
	// modifyTranslationToX(skeletonNodes);
	// FbxUtility::transFbxFile("E:\\work\\motion\\nobody.FBX", "E:\\work\\motion\\nobody_trans.FBX", skeletonNodes);

	int upAxis = 2;
	m_guiHelper->setUpAxis(upAxis);

	createEmptyDynamicsWorld();
	btVector3 gravity(0, 0, 0);
	// gravity[upAxis] = -9.8;
	m_dynamicsWorld->setGravity(gravity);

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	const btVector3 boneHalfExtents = btVector3(0.05, 0.4, 0.1);
	if (useFBX)
	{
		if (false)
		{
			std::vector<SkeletonNode> temp;
			bool useSub = false;
			if (useSub)
			{
				for (int i = 0; i < 6; i++)
				{
					temp.push_back(skeletonNodes[i]);
				}
			}
			else
			{
				for (int i = 0; i < 6; i++)
				{
					SkeletonNode node;
					node.idx = i;
					node.parentIdx = i - 1;
					if (i >= 2)
					{
						node.translation[2] = (i + 1) * 0.2;
						node.rotation[0] = 45;
					}
					temp.push_back(node);
				}
			}
			skeletonNodes.swap(temp);
		}
		if (true)
		{
			skeletonNodes[0].translation[0] = 0;
			skeletonNodes[0].translation[1] = 0;
			skeletonNodes[0].translation[2] = 3;
			/*
			skeletonNodes[0].rotation[0] = 0;
			skeletonNodes[0].rotation[1] = 0;
			skeletonNodes[0].rotation[2] = 0;
			skeletonNodes[1].translation[0] = 0;
			skeletonNodes[1].translation[1] = 0;
			skeletonNodes[1].translation[2] = 0;
			skeletonNodes[1].rotation[0] = 0;
			skeletonNodes[1].rotation[1] = 0;
			skeletonNodes[1].rotation[2] = 0;
			*/
		}
		
		m_multiBody = createMultiBodyFromSkeletonNodes(skeletonNodes);
		::createMultiBodyColliders(m_dynamicsWorld, m_multiBody);
	}
	else
	{
		m_multiBody = createTestMultiBody(boneHalfExtents);
		createMultiBodyColliders(m_dynamicsWorld, m_multiBody, boneHalfExtents);
	}
	m_dynamicsWorld->addMultiBody(m_multiBody);
	m_inverseModel = createInverseDynamicModel(m_multiBody);
	m_inverseModel->setGravityInWorldFrame(gravity);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void InverseDynamicsTest::stepSimulation(float deltaTime)
{
	return;
	// step the simulation
	if (m_dynamicsWorld)
	{
		bool enable = false;
		if (enable)
		{
			const int numDofs = m_multiBody->getNumDofs();
			btInverseDynamics::vecx nu(numDofs), qdot(numDofs), q(numDofs), jointForce(numDofs);
			for (int i = 0; i < numDofs; i++)
			{
				q[i] = m_multiBody->getJointPos(i);
				qdot[i] = m_multiBody->getJointVel(i);
				btScalar acceleration = 0;
				nu[i] = acceleration;
			}

			m_inverseModel->calculateInverseDynamics(q, qdot, nu, &jointForce);
			for (int i = 0; i < numDofs; i++)
			{
				m_multiBody->addJointTorque(i, jointForce[i]);
			}
		}
		m_dynamicsWorld->stepSimulation(deltaTime);
		// todo(thomas) check that this is correct:
		// want to advance by 10ms, with 1ms timesteps

		// m_dynamicsWorld->stepSimulation(1e-3, 0);  //,1e-3);
	}
}

btMultiBody* InverseDynamicsTest::createTestMultiBody(const btVector3 &boneHalfExtents)
{
	int numLinks = 5;
	btScalar mass = 1.0;
	btVector3 inertiaDiag; // todo，这是算的对角线转动惯量？？
	calcBoxShapeInertia(boneHalfExtents, mass, inertiaDiag);

	bool fixedBase = true;
	bool canSleep = false; // todo， what mean
	btMultiBody* multiBody = new btMultiBody(numLinks, mass, inertiaDiag, fixedBase, canSleep);
	btVector3 pos = btVector3(0, 3.f, 0);
	btQuaternion rotation = btQuaternion(0, 0, 0, 1);
	multiBody->setBasePos(pos);
	multiBody->setWorldToBaseRot(rotation);

	btVector3 parentComToCurrentCom(0, -boneHalfExtents[1] * 2.f, 0);                      //par body's COM to cur body's COM offset
	btVector3 currentPivotToCurrentCom(0, -boneHalfExtents[1], 0);                         //cur body's COM to cur body's PIV offset
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;  //par body's COM to cur body's PIV offset

	bool disableParentCollision = true; // todo， what mean

	btVector3 hingeJointAxis(1, 0, 0);
	for (int i = 0; i < numLinks; ++i)
	{
		multiBody->setupRevolute(i, mass, inertiaDiag, i - 1, btQuaternion(0, 0, 0, 1), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, disableParentCollision);
		//multiBody->setupSpherical(i, mass, inertiaDiag, i - 1, btQuaternion(0, 0, 0, 1), parentComToCurrentPivot, currentPivotToCurrentCom, disableParentCollision);
	}
	
	multiBody->finalizeMultiDof();

	multiBody->setHasSelfCollision(true);

	//multiBody->setLinearDamping(0.1f);
	//multiBody->setAngularDamping(0.9f);
	multiBody->setLinearDamping(0.0f);
	multiBody->setAngularDamping(0.0f);

	// 初始化的位置
	btScalar initDegree = 75.0;
	initDegree = 0;
	btQuaternion initQ(0, 0, initDegree);
	multiBody->setJointPosMultiDof(0, initQ);
	for (int i = 0; i < multiBody->getNumDofs(); i++)
	{
		multiBody->setJointVel(i, 0.6);
	}

	return multiBody;
}

void InverseDynamicsTest::createMultiBodyColliders(btMultiBodyDynamicsWorld *world, btMultiBody *multiBody, const btVector3 &boneHalfExtents)
{
	btAlignedObjectArray<btQuaternion> world_to_local;
	world_to_local.resize(multiBody->getNumLinks() + 1);

	btAlignedObjectArray<btVector3> local_origin;
	local_origin.resize(multiBody->getNumLinks() + 1);
	world_to_local[0] = multiBody->getWorldToBaseRot();
	local_origin[0] = multiBody->getBasePos();

	const btQuaternion &baseRotation = multiBody->getWorldToBaseRot();
	const btVector3 &basePos = multiBody->getBasePos();
	btBoxShape *baseBox = new btBoxShape(boneHalfExtents);
	btMultiBodyLinkCollider* baseCollider = new btMultiBodyLinkCollider(multiBody, -1);
	// todo shape、 collider的内存??
	baseCollider->setCollisionShape(baseBox);
	// todo friction ??
	float friction = 1;
	baseCollider->setFriction(friction);
	// todo demo这里rotate 角度用的负的？
	baseCollider->setWorldTransform(btTransform(baseRotation, basePos));

	multiBody->setBaseCollider(baseCollider);

	world->addCollisionObject(baseCollider);

	btQuaternion curRotation = baseRotation;
	btVector3 curPos = basePos;
	int numLinks = multiBody->getNumLinks();
	for (int i = 0; i < numLinks; ++i)
	{
		// todo shape、 collider的内存??
		btBoxShape *box = new btBoxShape(boneHalfExtents);
		btMultiBodyLinkCollider* collider = new btMultiBodyLinkCollider(multiBody, i);
		collider->setCollisionShape(box);
		collider->setFriction(friction);

		curRotation = multiBody->getParentToLocalRot(i) * curRotation;
		curPos = curPos + quatRotate(curRotation.inverse(), multiBody->getRVector(i));

		// todo demo这里rotate 角度用的负的？
		collider->setWorldTransform(btTransform(curRotation, curPos));

		multiBody->getLink(i).m_collider = collider;

		world->addCollisionObject(collider);
	}
}

void InverseDynamicsTest::calcForceWithKenimaticData(btInverseDynamics::MultiBodyTree* inverseModel)
{

}

btInverseDynamics::MultiBodyTree* InverseDynamicsTest::createInverseDynamicModel(btMultiBody *body)
{
	btInverseDynamics::btMultiBodyTreeCreator creator;
	if (-1 == creator.createFromBtMultiBody(body))
	{
		b3Error("error creating tree\n");
		return 0;
	}
	btInverseDynamics::MultiBodyTree* tree = btInverseDynamics::CreateMultiBodyTree(creator);
	return tree;
}

CommonExampleInterface* InverseDynamicsTestCreateFunc(CommonExampleOptions& options)
{
	return new InverseDynamicsTest(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(InverseDynamicsTestCreateFunc)
