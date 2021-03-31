#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include "Character.h"


const btVector3 UP_VECTOR2(0.0f, 1.0f, 0.0f);
const btVector3 ZERO_VECTOR2(0.0f, 0.0f, 0.0f);

class RayResultCallback2 : public btCollisionWorld::ClosestRayResultCallback{
public:
	RayResultCallback2() : btCollisionWorld::ClosestRayResultCallback(ZERO_VECTOR2, ZERO_VECTOR2){
	}

	btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace){
		//if (rayResult.m_collisionObject == mSelf) return 1.0f;
		//if (rayResult.m_collisionObject->getInternalType() == btCollisionObject::CO_GHOST_OBJECT) return 1.0f;
		return ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
	}

private:
	btCollisionObject* mSelf;
};

//=============================================================================
//=============================================================================
Character::Character() :
	onGround_(false),
	okToJump_(true),
	inAirTimer_(0.0f),
	jumpStarted_(false){
}

void Character::DelayedStart(){
	//collisionShape_ = node_->GetComponent<CollisionShape>(true);
	//kinematicController_ = node_->GetComponent<KinematicCharacterController>(true);
}

void Character::Start(){
	
}

void Character::FixedUpdate(float timeStep)
{
	// Update the in air timer. Reset if grounded
	if (!onGround_)
		inAirTimer_ += timeStep;
	else
		inAirTimer_ = 0.0f;
	// When character has been in air less than 1/10 second, it's still interpreted as being on ground
	bool softGrounded = inAirTimer_ < INAIR_THRESHOLD_TIME;

	// Update movement & animation
	const Matrix4f& rot = Matrix4f::IDENTITY;
	Vector3f moveDir = Vector3f(0.0f, 0.0f, 0.0f);
	onGround_ = kinematicController_->OnGround();

	/*if (controls_.IsDown(CTRL_FORWARD))
		moveDir += Vector3::FORWARD;
	if (controls_.IsDown(CTRL_BACK))
		moveDir += Vector3::BACK;
	if (controls_.IsDown(CTRL_LEFT))
		moveDir += Vector3::LEFT;
	if (controls_.IsDown(CTRL_RIGHT))
		moveDir += Vector3::RIGHT;*/

	// Normalize move vector so that diagonal strafing is not faster
	if (moveDir.lengthSq() > 0.0f)
		Vector3f::Normalize(moveDir);

	// rotate movedir
	curMoveDir_ = rot * moveDir;

	kinematicController_->SetWalkDirection(curMoveDir_ * (softGrounded ? MOVE_FORCE : INAIR_MOVE_FORCE));

	if (softGrounded){
		isJumping_ = false;
		// Jump. Must release jump control between jumps
		//if (controls_.IsDown(CTRL_JUMP))
		if (true){
			isJumping_ = true;
			if (okToJump_){
				okToJump_ = false;
				jumpStarted_ = true;
				kinematicController_->Jump();
			}

		}else{
			okToJump_ = true;
		}
	}
}