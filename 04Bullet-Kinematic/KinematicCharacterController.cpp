#include <iostream>

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletDynamics/Character/btKinematicCharacterController.h>

#include "KinematicCharacterController.h"

//=============================================================================
// NOTE! declare these function before #include <Urho3D/DebugNew> 
// otherwise you'll get a compile error for _DEBUG build
//=============================================================================
btPairCachingGhostObject* newPairCachingGhostObj()
{
	return new btPairCachingGhostObject();
}

btKinematicCharacterController* newKinematicCharCtrl(btPairCachingGhostObject *ghostCGO,
	btConvexShape *shape, float stepHeight, const btVector3 &upVec)
{
	return new btKinematicCharacterController(ghostCGO, shape, stepHeight, upVec);
}

//=============================================================================
//=============================================================================
const float STEP_HEIGHT = 0.4f;
const float JUMP_HEIGHT = 2.0f;
const float FALL_SPEED = 55.0f;
const float JUMP_SPEED = 9.0f;
const float MAX_SLOPE = 45.0f;
const float DEFAULT_DAMPING = 0.2f;
const Vector3f KINEMATIC_GRAVITY(0.0f, -100.0f, 0.0f);

//=============================================================================
//=============================================================================
KinematicCharacterController::KinematicCharacterController(btDynamicsWorld* physicsWorld)
	: colLayer_(1)
	, colMask_(0xffff)
	, gravity_(KINEMATIC_GRAVITY)
	, stepHeight_(STEP_HEIGHT)
	, maxJumpHeight_(JUMP_HEIGHT)
	, fallSpeed_(FALL_SPEED)
	, jumpSpeed_(JUMP_SPEED)
	, maxSlope_(MAX_SLOPE)
	, linearDamping_(DEFAULT_DAMPING)
	, angularDamping_(DEFAULT_DAMPING)
	, colShapeOffset_(Vector3f(0.0f, 0.0f, 0.0f))
	, reapplyAttributes_(false){

	

	pairCachingGhostObject_ = new btPairCachingGhostObject();
	pairCachingGhostObject_->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
	
	physicsWorld_ = physicsWorld;


}

KinematicCharacterController::~KinematicCharacterController(){
	ReleaseKinematic();
}

void KinematicCharacterController::ReleaseKinematic(){

	if (kinematicController_){
		RemoveKinematicFromWorld();
	}

	//kinematicController_.reset();
	//pairCachingGhostObject_.reset();
}

void KinematicCharacterController::AddKinematicToWorld(btCollisionShape* ghostShape){
		
		btTransform startTransform;
		startTransform.setOrigin(btVector3(btScalar(80.), btScalar(60.), btScalar(0.)));
		pairCachingGhostObject_->setWorldTransform(startTransform);

		pairCachingGhostObject_->setCollisionShape(ghostShape);			
		colShapeOffset_ = Vector3f(0.0f, 0.0f, 0.0f);
			
		kinematicController_ = newKinematicCharCtrl(pairCachingGhostObject_, dynamic_cast<btConvexShape*>(ghostShape), stepHeight_, btVector3(0.0f, 1.0f, 0.0f));

		// apply default settings
		ApplySettings();
		//btDiscreteDynamicsWorld *phyicsWorld = physicsWorld_;
		physicsWorld_->addCollisionObject(pairCachingGhostObject_, colLayer_, colMask_);
		physicsWorld_->addAction(kinematicController_);	
}


void KinematicCharacterController::ApplySettings(bool reapply){

	kinematicController_->setGravity(btVector3(KINEMATIC_GRAVITY[0], KINEMATIC_GRAVITY[1], KINEMATIC_GRAVITY[2]));
	kinematicController_->setLinearDamping(linearDamping_);
	kinematicController_->setAngularDamping(angularDamping_);
	kinematicController_->setStepHeight(stepHeight_);
	kinematicController_->setMaxJumpHeight(maxJumpHeight_);
	kinematicController_->setMaxSlope((PI *  maxSlope_) * (1.0f / 180.0f));
	kinematicController_->setJumpSpeed(jumpSpeed_);
	kinematicController_->setFallSpeed(fallSpeed_);

	if (reapply && pairCachingGhostObject_){
		btDynamicsWorld *phyicsWorld = physicsWorld_;
		phyicsWorld->removeCollisionObject(pairCachingGhostObject_);
		phyicsWorld->addCollisionObject(pairCachingGhostObject_, colLayer_, colMask_);
	}

	//SetTransform(node_->GetWorldPosition(), node_->GetWorldRotation());
}

void KinematicCharacterController::RemoveKinematicFromWorld(){

	if (kinematicController_ && physicsWorld_){
		btDynamicsWorld *phyicsWorld = physicsWorld_;
		phyicsWorld->removeCollisionObject(pairCachingGhostObject_);
		phyicsWorld->removeAction(kinematicController_);
	}
}

void KinematicCharacterController::SetCollisionLayer(int layer){

	if (physicsWorld_){
		if (layer != colLayer_){
			colLayer_ = layer;
			btDynamicsWorld *phyicsWorld = physicsWorld_;
			phyicsWorld->removeCollisionObject(pairCachingGhostObject_);
			phyicsWorld->addCollisionObject(pairCachingGhostObject_, colLayer_, colMask_);
		}
	}
}

void KinematicCharacterController::SetCollisionMask(int mask){

	if (physicsWorld_){
		if (mask != colMask_){
			colMask_ = mask;
			btDynamicsWorld *phyicsWorld = physicsWorld_;
			phyicsWorld->removeCollisionObject(pairCachingGhostObject_);
			phyicsWorld->addCollisionObject(pairCachingGhostObject_, colLayer_, colMask_);
		}
	}
}

void KinematicCharacterController::SetCollisionLayerAndMask(int layer, int mask){

	if (physicsWorld_){
		if (layer != colLayer_ || mask != colMask_){
			colLayer_ = layer;
			colMask_ = mask;
			btDynamicsWorld *phyicsWorld = physicsWorld_;
			phyicsWorld->removeCollisionObject(pairCachingGhostObject_);
			phyicsWorld->addCollisionObject(pairCachingGhostObject_, colLayer_, colMask_);
		}
	}
}

const Vector3f& KinematicCharacterController::GetPosition(){
	btVector3 pos = pairCachingGhostObject_->getWorldTransform().getOrigin();
	position_ = Vector3f(pos[0], pos[1], pos[2]) - colShapeOffset_;
	return position_;
}

const Quaternion& KinematicCharacterController::GetRotation(){
	btQuaternion quat = pairCachingGhostObject_->getWorldTransform().getRotation();
	rotation_ = Quaternion(quat[0], quat[1], quat[2], quat[3]);
	return rotation_;
}

void KinematicCharacterController::SetTransform(const Vector3f& position, const Quaternion& rotation){
	btTransform worldTrans;
	worldTrans.setIdentity();

	worldTrans.setRotation(btQuaternion(rotation[0], rotation[1], rotation[2], rotation[3]));
	worldTrans.setOrigin(btVector3(position[0], position[1], position[2]));
	pairCachingGhostObject_->setWorldTransform(worldTrans);
}

void KinematicCharacterController::GetTransform(Vector3f& position, Quaternion& rotation){

	btTransform worldTrans = pairCachingGhostObject_->getWorldTransform();
	btQuaternion quat = worldTrans.getRotation();
	rotation = Quaternion(quat[0], quat[1], quat[2], quat[3]);
	btVector3 pos = worldTrans.getOrigin();
	position = Vector3f(pos[0], pos[1], pos[2]);
}

void KinematicCharacterController::SetLinearDamping(float linearDamping){
	if (linearDamping != linearDamping_){
		linearDamping_ = linearDamping;
		kinematicController_->setLinearDamping(linearDamping_);
	}
}

void KinematicCharacterController::SetAngularDamping(float angularDamping){
	if (angularDamping != angularDamping_){
		angularDamping_ = angularDamping;
		kinematicController_->setAngularDamping(angularDamping_);
	}
}

void KinematicCharacterController::SetGravity(const Vector3f &gravity){
	if (gravity[0] != gravity_[0] && gravity[1] != gravity_[1] && gravity[2] != gravity_[2]){
		gravity_ = gravity;		
		kinematicController_->setGravity(btVector3(gravity_[0], gravity_[1], gravity_[2]));
	}
}

void KinematicCharacterController::SetStepHeight(float stepHeight){
	if (stepHeight != stepHeight_){
		stepHeight_ = stepHeight;
		kinematicController_->setStepHeight(stepHeight_);
	}
}

void KinematicCharacterController::SetMaxJumpHeight(float maxJumpHeight){
	if (maxJumpHeight != maxJumpHeight_){
		maxJumpHeight_ = maxJumpHeight;
		kinematicController_->setMaxJumpHeight(maxJumpHeight_);
	}
}

void KinematicCharacterController::SetFallSpeed(float fallSpeed){
	if (fallSpeed != fallSpeed_){
		fallSpeed_ = fallSpeed;
		kinematicController_->setFallSpeed(fallSpeed_);
	}
}

void KinematicCharacterController::SetJumpSpeed(float jumpSpeed){
	if (jumpSpeed != jumpSpeed_){
		jumpSpeed_ = jumpSpeed;
		kinematicController_->setJumpSpeed(jumpSpeed_);
	}
}

void KinematicCharacterController::SetMaxSlope(float maxSlope){
	if (maxSlope != maxSlope_){
		maxSlope_ = maxSlope;
		kinematicController_->setMaxSlope((PI *  maxSlope_) * (1.0f / 180.0f));
	}
}

void KinematicCharacterController::SetWalkDirection(const Vector3f& walkDir){
	kinematicController_->setWalkDirection(btVector3(walkDir[0], walkDir[1], walkDir[2]));
}

bool KinematicCharacterController::OnGround() const{
	return kinematicController_->onGround();
}

void KinematicCharacterController::Jump(const Vector3f &jump){
	kinematicController_->jump(btVector3(jump[0], jump[1], jump[2]));
}

bool KinematicCharacterController::CanJump() const{
	return kinematicController_->canJump();
}

void KinematicCharacterController::ApplyImpulse(const Vector3f &impulse){
	kinematicController_->applyImpulse(btVector3(impulse[0], impulse[1], impulse[2]));
}

void KinematicCharacterController::SetAngularVelocity(const Vector3f &velocity){
	kinematicController_->setAngularVelocity(btVector3(velocity[0], velocity[1], velocity[2]));
}

const Vector3f KinematicCharacterController::GetAngularVelocity() const{
	btVector3 angVel = kinematicController_->getAngularVelocity();
	return Vector3f(angVel[0], angVel[1], angVel[2]);
}

void KinematicCharacterController::SetLinearVelocity(const Vector3f &velocity){
	kinematicController_->setLinearVelocity(btVector3(velocity[0], velocity[1], velocity[2]));
}

const Vector3f KinematicCharacterController::GetLinearVelocity() const{
	btVector3 linVel = kinematicController_->getAngularVelocity();
	return Vector3f(linVel[0], linVel[1], linVel[2]);
}

void KinematicCharacterController::Warp(const Vector3f &position){
	kinematicController_->warp(btVector3(position[0], position[1], position[2]));
}