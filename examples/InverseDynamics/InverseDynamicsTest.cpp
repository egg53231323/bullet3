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
#include "SkeletonUtility.h"

class InverseDynamicsTest : public CommonMultiBodyBase
{
	btMultiBody* m_multiBody;
	btInverseDynamics::MultiBodyTree* m_inverseModel;

public:
	InverseDynamicsTest(struct GUIHelperInterface* helper);
	virtual ~InverseDynamicsTest();

	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);
	bool preprocessSkeletonNodesForTest(std::vector<SkeletonNode> &skeletonNodes);
	btMultiBody* createTestMultiBody(const btVector3 &boneHalfExtents);
	void createMultiBodyColliders(btMultiBodyDynamicsWorld *world, btMultiBody *body, const btVector3 &boneHalfExtents);
	btInverseDynamics::MultiBodyTree* createInverseDynamicModel(btMultiBody *body);
	void calcForceWithKenimaticData(btInverseDynamics::MultiBodyTree* inverseModel);

	virtual void resetCamera()
	{
		float dist = 1;
		float pitch = -20;
		pitch = 0;
		float yaw = 0;
		// float targetPos[3] = { 0, 30, 6 };
		float targetPos[3] = { 0, -30, 0 };
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

bool InverseDynamicsTest::preprocessSkeletonNodesForTest(std::vector<SkeletonNode> &skeletonNodes)
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

	skeletonNodes[0].translation[0] = 0;
	skeletonNodes[0].translation[1] = 0;
	skeletonNodes[0].translation[2] = 0;
	skeletonNodes[0].rotation[0] = 0;
	skeletonNodes[0].rotation[1] = 0;
	skeletonNodes[0].rotation[2] = 0;
	skeletonNodes[0].rotationPre[0] = 0;


	skeletonNodes[1].translation[0] = 0;
	skeletonNodes[1].translation[1] = 0;
	skeletonNodes[1].translation[2] = 0;
	skeletonNodes[1].rotation[0] = 0;
	skeletonNodes[1].rotation[1] = 0;
	skeletonNodes[1].rotation[2] = 0;

	return true;
}

void InverseDynamicsTest::initPhysics()
{
	bool useFBX = true;

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
		std::vector<SkeletonNode> skeletonNodes;
		FbxUtility::loadFbxFile("E:\\work\\motion\\nobody.FBX", skeletonNodes);

		preprocessSkeletonNodesForTest(skeletonNodes);
		FbxUtility::transFbxFile("E:\\work\\motion\\nobody.FBX", "E:\\work\\motion\\nobody_trans.FBX", skeletonNodes);

		m_multiBody = SkeletonUtility::createMultiBodyFromSkeletonNodes(skeletonNodes);
		SkeletonUtility::createMultiBodyColliders(m_dynamicsWorld, m_multiBody);
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
	SkeletonUtility::calcBoxShapeInertia(boneHalfExtents, mass, inertiaDiag);

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
