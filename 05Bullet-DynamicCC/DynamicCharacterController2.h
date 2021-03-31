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

	DynamicCharacterController2();
	DynamicCharacterController2(btRigidBody* rigidBody, btDiscreteDynamicsWorld* pPhysicsWorld, const Vector3f spawnPos, float radius, float height, float mass, float stepHeight, int collisionFilterGroup = 1, int collisionFilterMask = -1);
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


	void create(btMotionState* motionState, btDiscreteDynamicsWorld* physicsWorld, float mass, float radius, float height, int collisionFilterGroup = 1, int collisionFilterMask = -1, void* rigidBodyUserPointer = NULL);
	void create(btRigidBody* rigidBody, btDiscreteDynamicsWorld* physicsWorld, int collisionFilterGroup = 1, int collisionFilterMask = -1, void* rigidBodyUserPointer = NULL);
	void destroy();

	void setParameters(float maxClimbSlopeAngle);
	void setDistanceOffset(float value);

	void preStep(); // Call before the physics are stepped.
	void postStep(); // Call after the physics are stepped.
	bool isStepping() const;

	bool onGround() const;
	void jump(const btVector3& direction, float force);

	void setMovementXZ(const Vector2f& movementVector);
	void setMovementXYZ(const btVector3& movementVector);

	void setLinearVelocity(const btVector3& vel);
	const btVector3& getLinearVelocity();

	btRigidBody* getRigidBody() const { return m_pRigidBody; }
	btCollisionShape* getCollisionShape() const { return mShape; }

	void moveCharacterAlongY(float step);
	bool mOnGround;
	
	float mStepHeight;
	float mDistanceOffset;
	bool mIsStepping;


	float mCharacterMovementX;
	float mCharacterMovementZ;

	float mCharacterMovementY;
	bool movingUpward = true;
	btCollisionShape* mShape;
};

