#ifndef _KINEMATICCHARACTERCONTROLLER_H_INCLUDED_
#define _KINEMATICCHARACTERCONTROLLER_H_INCLUDED_

#include <memory>
#include <btBulletDynamicsCommon.h>
#include "Vector.h"

class btPairCachingGhostObject;
class btKinematicCharacterController;

//=============================================================================
//=============================================================================
class KinematicCharacterController{

public:
	KinematicCharacterController(btDynamicsWorld* physicsWorld);
	virtual ~KinematicCharacterController();

	const Vector3f& GetPosition();
	const Quaternion& GetRotation();
	void SetTransform(const Vector3f& position, const Quaternion& rotation);
	void GetTransform(Vector3f& position, Quaternion& rotation);

	void SetCollisionLayer(int layer);
	void SetCollisionMask(int mask);
	void SetCollisionLayerAndMask(int layer, int mask);

	void SetGravity(const Vector3f& gravity);
	const Vector3f& GetGravity() const { return gravity_; }
	void SetLinearDamping(float linearDamping);
	float GetLinearDamping() const { return linearDamping_; }
	void SetAngularDamping(float angularDamping);
	float GetAngularDamping() const { return angularDamping_; }

	void SetStepHeight(float stepHeight);
	float GetStepHeight() const { return stepHeight_; }
	void SetMaxJumpHeight(float maxJumpHeight);
	float GetMaxJumpHeight() const { return maxJumpHeight_; }
	void SetFallSpeed(float fallSpeed);
	float GetFallSpeed() const { return fallSpeed_; }
	void SetJumpSpeed(float jumpSpeed);
	float GetJumpSpeed() const { return jumpSpeed_; }
	void SetMaxSlope(float maxSlope);
	float GetMaxSlope() const { return maxSlope_; }

	void SetWalkDirection(const Vector3f& walkDir);
	bool OnGround() const;
	void Jump(const Vector3f &jump = Vector3f(0.0f, 0.0f, 0.0f));
	/// ApplyImpulse is same as Jump
	void ApplyImpulse(const Vector3f &impulse);
	bool CanJump() const;

	void SetAngularVelocity(const Vector3f &velocity);
	const Vector3f GetAngularVelocity() const;
	void SetLinearVelocity(const Vector3f &velocity);
	const Vector3f GetLinearVelocity() const;
	void Warp(const Vector3f &position);


	void ReleaseKinematic();
	void ApplySettings(bool reapply = false);
	
	void AddKinematicToWorld(btCollisionShape* ghostShape);
	void RemoveKinematicFromWorld();


	int colLayer_;
	int colMask_;
	float stepHeight_;
	float maxJumpHeight_;
	float jumpSpeed_;
	float fallSpeed_;
	float maxSlope_;
	float linearDamping_;
	float angularDamping_;
	Vector3f gravity_;

	//WeakPtr<PhysicsWorld> physicsWorld_;
	btDynamicsWorld* physicsWorld_;
	btPairCachingGhostObject* pairCachingGhostObject_;
	btKinematicCharacterController* kinematicController_;

	
	
	Vector3f position_;
	Quaternion rotation_;
	Vector3f colShapeOffset_;
	bool reapplyAttributes_;

};

#endif