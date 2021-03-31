#include <iostream>
#include "DynamicCharacterController2.h"

class IgnoreBodyAndGhostCast : public btCollisionWorld::ClosestRayResultCallback{

private:
	btRigidBody* m_pBody;
	btPairCachingGhostObject* m_pGhostObject;

public:
	IgnoreBodyAndGhostCast(btRigidBody* pBody, btPairCachingGhostObject* pGhostObject, int collisionFilterGroup = 1, int collisionFilterMask = -1)
		: btCollisionWorld::ClosestRayResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0)),
		m_pBody(pBody), m_pGhostObject(pGhostObject)
	{
		this->m_collisionFilterGroup = collisionFilterGroup;
		this->m_collisionFilterMask = collisionFilterMask;
	}

	btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
	{
		
		if (rayResult.m_collisionObject == m_pBody || rayResult.m_collisionObject == m_pGhostObject) {
			
			return 1.0f;
		}
		return ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
	}
};

DynamicCharacterController2::DynamicCharacterController2(btDiscreteDynamicsWorld* pPhysicsWorld, const Vector3f spawnPos, float radius, float height, float mass, float stepHeight, int collisionFilterGroup, int collisionFilterMask)
	: m_pPhysicsWorld(pPhysicsWorld), m_bottomYOffset(height / 2.0f + radius), m_bottomRoundedRegionYOffset((height + radius) / 2.0f),
	m_deceleration(0.1f), m_maxSpeed(50.0f), m_jumpImpulse(600.0f),
	m_manualVelocity(0.0f, 0.0f, 0.0f), m_onGround(false), m_hittingWall(false),
	m_jumpRechargeTimer(0.0f), m_jumpRechargeTime(24.0f), m_stepHeight(stepHeight), mOnSteepSlope(false), mMaxClimbSlopeAngle(40.0f), m_friction(40.0f)
{
	mSlopeNormal.setZero();
	mCollisionFilterGroup = collisionFilterGroup;
	mCollisionFilterMask = collisionFilterMask;

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(spawnPos[0], spawnPos[1], spawnPos[2]));
	m_pMotionState = new btDefaultMotionState(transform);

	m_pCollisionShape = new btCapsuleShape(radius, height);

	

	btVector3 intertia;
	m_pCollisionShape->calculateLocalInertia(mass, intertia);

	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, m_pMotionState, m_pCollisionShape, intertia);

	// No friction, this is done manually
	rigidBodyCI.m_friction = m_friction;
	//rigidBodyCI.m_additionalDamping = true;
	//rigidBodyCI.m_additionalLinearDampingThresholdSqr= 1.0f;
	//rigidBodyCI.m_additionalLinearDampingThresholdSqr = 0.5f;
	rigidBodyCI.m_restitution = 0.0f;

	rigidBodyCI.m_linearDamping = 0.0f;

	m_pRigidBody = new btRigidBody(rigidBodyCI);

	// Keep upright
	m_pRigidBody->setAngularFactor(0.0f);

	// No sleeping (or else setLinearVelocity won't work)
	m_pRigidBody->setActivationState(DISABLE_DEACTIVATION);

	m_pPhysicsWorld->addRigidBody(m_pRigidBody, collisionFilterGroup, collisionFilterMask);

	// Ghost object that is synchronized with rigid body
	m_pGhostObject = new btPairCachingGhostObject();

	m_pGhostObject->setCollisionShape(m_pCollisionShape);
	m_pGhostObject->setUserPointer(this);
	m_pGhostObject->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);

	// Specify filters manually, otherwise ghost doesn't collide with statics for some reason
	m_pPhysicsWorld->addCollisionObject(m_pGhostObject, collisionFilterGroup, collisionFilterMask);
}

DynamicCharacterController2::~DynamicCharacterController2(){
	
	m_pPhysicsWorld->removeRigidBody(m_pRigidBody);
	m_pPhysicsWorld->removeCollisionObject(m_pGhostObject);
	
	delete m_pCollisionShape;
	delete m_pMotionState;
	delete m_pRigidBody;

	delete m_pGhostObject;
}

void DynamicCharacterController2::Walk(const Vector2f dir)
{
	Vector2f velocityXZ(dir + Vector2f(m_manualVelocity[0], m_manualVelocity[2]));

	// Prevent from going over maximum speed
	float speedXZ = velocityXZ.length();

	if (speedXZ > m_maxSpeed)
		velocityXZ = velocityXZ / speedXZ * m_maxSpeed;

	m_manualVelocity[0] = velocityXZ[0];
	m_manualVelocity[2] = velocityXZ[1];

	UpdateVelocity();
}

void DynamicCharacterController2::setVelocityXZ(const Vector2f dir) {
	m_manualVelocity[0] = dir[0];
	m_manualVelocity[2] = dir[1];
}

void DynamicCharacterController2::Walk(const Vector3f dir){
	Walk(Vector2f(dir[0], dir[2]));
}

void DynamicCharacterController2::Update(float deltaTime){
	// Synch ghost with actually object
	m_pGhostObject->setWorldTransform(m_pRigidBody->getWorldTransform());
	//m_pGhostObject->getWorldTransform().getOrigin().setY(m_pGhostObject->getWorldTransform().getOrigin().getY() - 0.01f);

	// Update transform
	m_pMotionState->getWorldTransform(m_motionTransform);

	m_onGround = false;

	ParseGhostContacts();

	UpdatePosition();
	
	// Update jump timer
	if (m_jumpRechargeTimer < m_jumpRechargeTime)
		m_jumpRechargeTimer += deltaTime;
}

void DynamicCharacterController2::ParseGhostContacts(){
	btManifoldArray manifoldArray;
	btBroadphasePairArray &pairArray = m_pGhostObject->getOverlappingPairCache()->getOverlappingPairArray();
	int numPairs = pairArray.size();

	// Set false now, may be set true in test
	m_hittingWall = false;

	m_surfaceHitNormals.clear();

	for (int i = 0; i < numPairs; i++){
		manifoldArray.clear();

		const btBroadphasePair &pair = pairArray[i];

		btBroadphasePair* collisionPair = m_pPhysicsWorld->getPairCache()->findPair(pair.m_pProxy0, pair.m_pProxy1);

		if (collisionPair == NULL)
		continue;

		if (collisionPair->m_algorithm != NULL)
		collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);

		for (int j = 0; j < manifoldArray.size(); j++){
			btPersistentManifold* pManifold = manifoldArray[j];

			// Skip the rigid body the ghost monitors
			if (pManifold->getBody0() == m_pRigidBody)
				continue;

			for (int p = 0; p < pManifold->getNumContacts(); p++){

				const btManifoldPoint &point = pManifold->getContactPoint(p);

				if (point.getDistance() < 0.0f){
					//const btVector3 &ptA = point.getPositionWorldOnA();
					const btVector3 &ptB = point.getPositionWorldOnB();

					//const btVector3 &normalOnB = point.m_normalWorldOnB;

					// If point is in rounded bottom region of capsule shape, it is on the ground
					if (ptB.getY() < m_motionTransform.getOrigin().getY() - m_bottomRoundedRegionYOffset)
						m_onGround = true;
					else{
						m_hittingWall = true;
						m_surfaceHitNormals.push_back(Vector3f(point.m_normalWorldOnB[0], point.m_normalWorldOnB[1], point.m_normalWorldOnB[2]));
					}
				}
			}
		}
	}
}

void DynamicCharacterController2::UpdateVelocity()
{
	// Adjust only xz velocity
	m_manualVelocity[1] = m_pRigidBody->getLinearVelocity().getY();

	m_pRigidBody->setLinearVelocity(btVector3(m_manualVelocity[0], m_manualVelocity[1], m_manualVelocity[2]));

	// Decelerate
	//m_manualVelocity -= m_manualVelocity * m_deceleration * m_pPhysicsWorld->GetScene()->m_frameTimer.GetTimeMultiplier();
	m_manualVelocity -= m_manualVelocity * m_deceleration;
	if (m_hittingWall){
		for (unsigned int i = 0, size = m_surfaceHitNormals.size(); i < size; i++){
			// Cancel velocity across normal			
			Vector3f velInNormalDir((Vector3f::Dot(m_manualVelocity, m_surfaceHitNormals[i]) *(1.0f / m_surfaceHitNormals[i].lengthSq())) * m_surfaceHitNormals[i]);

			// Apply correction
			m_manualVelocity -= velInNormalDir * 1.05f;
		}
		// Do not adjust rigid body velocity manually (so bodies can still be pushed by character)
		return;
	}
}

void DynamicCharacterController2::UpdatePosition(){
	btVector3 from = m_pRigidBody->getWorldTransform().getOrigin();
	btVector3 to = from - btVector3(0.0f, m_bottomYOffset + m_stepHeight, 0.0f);
	// Ray cast, ignore rigid body
	IgnoreBodyAndGhostCast rayCallBack_bottom(m_pRigidBody, m_pGhostObject, mCollisionFilterGroup, mCollisionFilterMask);
	m_pPhysicsWorld->rayTest(from, to, rayCallBack_bottom);
	
	// Bump up if hit
	if (rayCallBack_bottom.hasHit()){
		//m_pRigidBody->setFriction(m_friction);
		float previousY = m_pRigidBody->getWorldTransform().getOrigin().getY();
		
		//m_pRigidBody->getWorldTransform().getOrigin().setY(previousY + (m_bottomYOffset + m_stepHeight) * (1.0f - rayCallBack_bottom.m_closestHitFraction));
		m_pRigidBody->getWorldTransform().getOrigin().setY(previousY + (m_stepHeight) * (1.0f - rayCallBack_bottom.m_closestHitFraction));
		btVector3 vel(m_pRigidBody->getLinearVelocity());
	
		//vel.setY(0.0f);
		m_pRigidBody->setLinearVelocity(vel);

		m_onGround = true;	
	}

	float testOffset = 0.07f;

	// Ray cast, ignore rigid body
	IgnoreBodyAndGhostCast rayCallBack_top(m_pRigidBody, m_pGhostObject, mCollisionFilterGroup, mCollisionFilterMask);

	m_pPhysicsWorld->rayTest(m_pRigidBody->getWorldTransform().getOrigin(), m_pRigidBody->getWorldTransform().getOrigin() + btVector3(0.0f, m_bottomYOffset + testOffset, 0.0f), rayCallBack_top);

	// Bump up if hit
	if (rayCallBack_top.hasHit()){
		m_pRigidBody->getWorldTransform().setOrigin(m_previousPosition);

		btVector3 vel(m_pRigidBody->getLinearVelocity());

		vel.setY(0.0f);

		m_pRigidBody->setLinearVelocity(vel);
	}

	m_previousPosition = m_pRigidBody->getWorldTransform().getOrigin();
}

void DynamicCharacterController2::Jump(){
	
	if (m_onGround && m_jumpRechargeTimer >= m_jumpRechargeTime){
		m_jumpRechargeTimer = 0.0f;
		m_pRigidBody->applyCentralImpulse(btVector3(0.0f, m_jumpImpulse, 0.0f));

		// Move upwards slightly so velocity isn't immediately canceled when it detects it as on ground next frame
		const float jumpYOffset = 0.01f;
		float previousY = m_pRigidBody->getWorldTransform().getOrigin().getY();
		m_pRigidBody->getWorldTransform().getOrigin().setY(previousY + jumpYOffset);
	}
}

void DynamicCharacterController2::Jump(const btVector3& direction, float force) {

	if (m_onGround) {
		m_onGround = false;
		m_pRigidBody->applyCentralImpulse(direction * force);
	}
}

Vector3f DynamicCharacterController2::GetPosition() const{
	btVector3 pos = m_motionTransform.getOrigin();	
	return Vector3f(pos[0], pos[1], pos[2]);
}

Vector3f DynamicCharacterController2::GetVelocity() const{
	btVector3 linVel = m_pRigidBody->getLinearVelocity();
	return Vector3f(linVel[0], linVel[1], linVel[2]);
}

bool DynamicCharacterController2::IsOnGround() const{
	return m_onGround;
}

void DynamicCharacterController2::setSlopeAngle(float degrees) {
	mMaxClimbSlopeAngle = cos(degrees * PI * (1.0f / 180.0f));
}
