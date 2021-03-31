#pragma once

#include <vector>

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>


#include"Vector.h"

class DynamicCharacterController2
{
public:
	// Physics
	btDiscreteDynamicsWorld* m_pPhysicsWorld;
	
	btCollisionShape* m_pCollisionShape;
	btDefaultMotionState* m_pMotionState;
	btRigidBody* m_pRigidBody;
	btPairCachingGhostObject* m_pGhostObject;

	bool m_onGround;
	//bool m_onJumpableGround; // A bit lower contact than just onGround
	bool m_hittingWall;

	float m_bottomYOffset;
	float m_bottomRoundedRegionYOffset;

	float m_stepHeight;

	btTransform m_motionTransform;

	Vector3f m_manualVelocity;
	std::vector<Vector3f> m_surfaceHitNormals;

	btVector3 m_previousPosition;

	float m_jumpRechargeTimer;

	void ParseGhostContacts();

	void UpdatePosition();
	void UpdateVelocity();


	float m_deceleration;
	float m_maxSpeed;
	float m_jumpImpulse;

	float m_jumpRechargeTime;

	float m_friction;

	bool mOnSteepSlope;
	btVector3 mSlopeNormal;
	float mMaxClimbSlopeAngle;


	int mCollisionFilterGroup;
	int mCollisionFilterMask;

	DynamicCharacterController2(btDiscreteDynamicsWorld* pPhysicsWorld, const Vector3f spawnPos, float radius, float height, float mass, float stepHeight, int collisionFilterGroup = 1, int collisionFilterMask = -1);
	~DynamicCharacterController2();

	// Acceleration vector in XZ plane
	void Walk(const Vector2f dir);
	void setVelocityXZ(const Vector2f dir);
	void setSlopeAngle(float degrees);

	// Ignores y
	void Walk(const Vector3f dir);

	void Update(float deltaTime);

	void Jump();
	void Jump(const btVector3& direction, float force);

	Vector3f GetPosition() const;
	Vector3f GetVelocity() const;

	bool IsOnGround() const;
};

