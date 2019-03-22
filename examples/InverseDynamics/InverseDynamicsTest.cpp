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

class InverseDynamicsTest : public CommonMultiBodyBase
{
	btMultiBody* m_multiBody;

public:
	InverseDynamicsTest(struct GUIHelperInterface* helper);
	virtual ~InverseDynamicsTest();

	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);
	btMultiBody* createTestMultiBody(const btVector3 &boneHalfExtents);
	void InverseDynamicsTest::createMultiBodyColliders(btMultiBodyDynamicsWorld *world, btMultiBody *body, const btVector3 &boneHalfExtents);
	void calcBoxShapeInteria(const btVector3 &halfExtents, btScalar mass, btVector3 &interia);

	virtual void resetCamera()
	{
		float dist = 1;
		float pitch = -35;
		float yaw = 50;
		float targetPos[3] = { -3, 2.8, -2.5 };
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
	//roboticists like Z up
	int upAxis = 2;
	upAxis = 1;
	m_guiHelper->setUpAxis(upAxis);

	createEmptyDynamicsWorld();
	btVector3 gravity(0, 0, 0);
	gravity[upAxis] = -9.8;
	m_dynamicsWorld->setGravity(gravity);

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	const btVector3 boneHalfExtents = btVector3(0.05, 0.37, 0.1);
	m_multiBody = createTestMultiBody(boneHalfExtents);
	m_dynamicsWorld->addMultiBody(m_multiBody);
	createMultiBodyColliders(m_dynamicsWorld, m_multiBody, boneHalfExtents);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void InverseDynamicsTest::stepSimulation(float deltaTime)
{

	// step the simulation
	if (m_dynamicsWorld)
	{
		float internalTimeStep = 1. / 240.f;
		m_dynamicsWorld->stepSimulation(deltaTime, 10, internalTimeStep);
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
	calcBoxShapeInteria(boneHalfExtents, mass, inertiaDiag);

	bool fixedBase = true;
	bool canSleep = false; // todo， what mean
	btMultiBody* multiBody = new btMultiBody(numLinks, mass, inertiaDiag, fixedBase, canSleep);
	btVector3 pos = btVector3(-0.4f, 3.f, 0.f);
	btQuaternion rotation = btQuaternion(0, 0, 0, 1);
	multiBody->setBasePos(pos);
	multiBody->setWorldToBaseRot(rotation);

	btVector3 parentComToCurrentCom(0, -boneHalfExtents[1] * 2.f, 0);                      //par body's COM to cur body's COM offset
	btVector3 currentPivotToCurrentCom(0, -boneHalfExtents[1], 0);                         //cur body's COM to cur body's PIV offset
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;  //par body's COM to cur body's PIV offset

	bool disableParentCollision = true; // todo， what mean

	for (int i = 0; i < numLinks; ++i)
	{
		multiBody->setupSpherical(i, mass, inertiaDiag, i - 1, btQuaternion(0, 0, 0, 1), parentComToCurrentPivot, currentPivotToCurrentCom, disableParentCollision);
	}
	
	multiBody->finalizeMultiDof();

	multiBody->setHasSelfCollision(true);

	multiBody->setLinearDamping(0.1f);
	multiBody->setAngularDamping(0.9f);

	// 初始化的位置
	btScalar initDegree = 75.0;
	btQuaternion initQ(0, 0, initDegree);
	multiBody->setJointPosMultiDof(0, initQ);

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

void InverseDynamicsTest::calcBoxShapeInteria(const btVector3 &halfExtents, btScalar mass, btVector3 &interia)
{
	btBoxShape shape = btBoxShape(halfExtents);
	shape.calculateLocalInertia(mass, interia);
}

CommonExampleInterface* InverseDynamicsTestCreateFunc(CommonExampleOptions& options)
{
	return new InverseDynamicsTest(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(InverseDynamicsTestCreateFunc)
