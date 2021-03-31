#ifndef _CHARACTER_H_INCLUDED_
#define _CHARACTER_H_INCLUDED_

#include <memory>
#include "KinematicCharacterController.h"

class btPairCachingGhostObject;

//=============================================================================
//=============================================================================
const int CTRL_FORWARD = 1;
const int CTRL_BACK = 2;
const int CTRL_LEFT = 4;
const int CTRL_RIGHT = 8;
const int CTRL_JUMP = 16;

const float MOVE_FORCE = 0.2f;
const float INAIR_MOVE_FORCE = 0.06f;
const float BRAKE_FORCE = 0.2f;
const float JUMP_FORCE = 7.0f;
const float YAW_SENSITIVITY = 0.1f;
const float INAIR_THRESHOLD_TIME = 0.1f;


class Character{

public:
	/// Construct.
	Character();

	/// Register object factory and attributes.
	static void RegisterObject();

	void DelayedStart();

	/// Handle startup. Called by LogicComponent base class.
	void Start();
	/// Handle physics world update. Called by LogicComponent base class.
	void FixedUpdate(float timeStep);
	void FixedPostUpdate(float timeStep);

protected:
	bool IsNodeMovingPlatform() const;
	void NodeOnMovingPlatform();
	/// Handle physics collision event.
	//void HandleNodeCollision(StringHash eventType, VariantMap& eventData);

	bool onGround_;
	bool okToJump_;
	float inAirTimer_;

	// extra vars
	Vector3f curMoveDir_;
	bool isJumping_;
	bool jumpStarted_;

	std::weak_ptr<btCollisionShape> collisionShape_;
	//std::weak_ptr<KinematicCharacterController> kinematicController_;
	KinematicCharacterController* kinematicController_;

	btCollisionWorld* mCollisionWorld;
	btPairCachingGhostObject* pairCachingGhostObject_;
};

#endif