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
#include "AnimationUtility.h"

class InverseDynamicsTest : public CommonMultiBodyBase
{
	btMultiBody* m_multiBody;
	btInverseDynamics::MultiBodyTree* m_inverseModel;
	int m_stepCount;
	float m_timeCount;
	std::vector<SkeletonNode> m_skeletonNodes;
	std::vector<btQuaternion> m_zeroNodeWorldToLocalRotations;
	std::vector<btQuaternion> m_zeroJointFrameRotations;

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
		float targetPos[3] = { 0, -10, 2 };
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

InverseDynamicsTest::InverseDynamicsTest(struct GUIHelperInterface* helper)
	: CommonMultiBodyBase(helper),
	m_multiBody(0),
	m_stepCount(0),
	m_timeCount(0)
{

}

InverseDynamicsTest::~InverseDynamicsTest()
{

}

void getSubNodes(std::vector<SkeletonNode> &skeletonNodes, int count)
{
	int maxCount = (int)skeletonNodes.size();
	if (count > maxCount || count < 0)
	{
		return;
	}
	std::vector<SkeletonNode> temp;
	for (int i = 0; i < count; i++)
	{
		temp.push_back(skeletonNodes[i]);
	}
	skeletonNodes.swap(temp);
}

bool InverseDynamicsTest::preprocessSkeletonNodesForTest(std::vector<SkeletonNode> &skeletonNodes)
{
#if 1
	int skeletonNodeCount = (int)skeletonNodes.size();

	if (true)
	{
		for (int i = 0; i < skeletonNodeCount; i++)
		{
			SkeletonNode &node = skeletonNodes[i];
			//Bip01 Spine
			//Bip01 L Thigh
			//Bip01 R Thigh
			if (i == 0)
			{
				node.animationR[0].clear();
				node.animationR[1].clear();
				node.animationR[2].clear();
				node.animationR[0].addKey(0, 0);
				//node.animationR[0].addKey(3000, 45);
			}
			else if (i == 1)
			{
				node.animationR[0].clear();
				node.animationR[1].clear();
				node.animationR[2].clear();
				node.animationR[1].addKey(0, 0);
				node.animationR[1].addKey(3000, 90);
			}
			else
			{
				//node.animationR[0].clear();
				//node.animationR[1].clear();
				//node.animationR[2].clear();
			}
		}
	}


	skeletonNodes[0].setTranslation(0, 0, 0);
	skeletonNodes[0].setRotation(0, 0, 0);
	skeletonNodes[0].setRotationPre(0, 0, 0);

	skeletonNodes[1].setTranslation(0, 0, 0);
	skeletonNodes[1].setRotation(0, 0, 0);
#endif
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
		FbxUtility::loadFbxFile("D:\\motion\\bullet\\nobody.FBX", m_skeletonNodes);

		preprocessSkeletonNodesForTest(m_skeletonNodes);
		FbxUtility::transFbxFile("D:\\motion\\bullet\\nobody.FBX", "E:\\work\\motion\\nobody_trans.FBX", m_skeletonNodes);

		m_multiBody = SkeletonUtility::createMultiBodyFromSkeletonNodes(m_skeletonNodes, m_zeroNodeWorldToLocalRotations, m_zeroJointFrameRotations);
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
	m_stepCount++;
	m_timeCount += deltaTime;
	// step the simulation
	if (m_dynamicsWorld)
	{
		AnimationKeyTime time = int(m_timeCount * 1000) % 3000;
		std::vector<btQuaternion> worldToLocalRoations, jointRotations;
		SkeletonUtility::calcNodeWorldToLocalRotations(m_skeletonNodes, worldToLocalRoations, time);
		SkeletonUtility::calcJointRotationsAtTime(m_skeletonNodes, time, m_zeroJointFrameRotations, jointRotations);

		SkeletonNode::AnimValueType x = 0, y = 0, z = 0;
		m_skeletonNodes[0].getTranslationAtTime(time, x, y, z);
		m_multiBody->setBasePos(btVector3(x, y, z));
		btQuaternion baseRotation = SkeletonUtility::skeletonNodeRotation(m_skeletonNodes[0], time) * SkeletonUtility::skeletonNodeRotation(m_skeletonNodes[1]);
		m_multiBody->setWorldToBaseRot(baseRotation);

		int count = (int)jointRotations.size();
		for (int i = 2; i < count; i++)
		{
			const SkeletonNode &node = m_skeletonNodes[i];
			btQuaternion curRotaion = jointRotations[i];
			float pos[4] = { curRotaion.x(), curRotaion.y(), curRotaion.z(), curRotaion.w()};
			m_multiBody->setJointPosMultiDof(i - 2, pos);
		}

		/*
		btScalar degree = m_stepCount / 10.0;
		degree = 45;
		float pos[4] = { 0, btSin(SkeletonUtility::degreeToRad(degree / 2)), 0, btCos(SkeletonUtility::degreeToRad(degree/ 2))};
		m_multiBody->setJointPosMultiDof(0, pos);
		*/

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
