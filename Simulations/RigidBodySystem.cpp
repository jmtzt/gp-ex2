#include "rigidBodySystem.h"

RigidBodySystem::RigidBodySystem()
{
	rigidBodies = vector<RigidBody>();
	forcesOnAllRigidBodies = vector<AppliedForce>();
}

void RigidBodySystem::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	AppliedForce forceOnRigidBody = AppliedForce();
	forceOnRigidBody.rigidBodyIndex = i;
	forceOnRigidBody.position = loc;
	forceOnRigidBody.force = force;
	forcesOnAllRigidBodies.push_back(forceOnRigidBody);
}

void RigidBodySystem::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody rigidBody = RigidBody(position, size, mass);
	rigidBodies.push_back(rigidBody);
}

void RigidBodySystem::setOrientationOf(int i, Quat orientation)
{
	rigidBodies[i].orientation = orientation;
}

void RigidBodySystem::setVelocityOf(int i, Vec3 velocity)
{
	rigidBodies[i].linearVel = velocity;
}

void RigidBodySystem::resetForces()
{
	forcesOnAllRigidBodies = vector<AppliedForce>();
}

vector<AppliedForce> RigidBodySystem::allForcesOnRigidBody(int i)
{
	/// <summary>
	/// TODO: SUM OVER ALL FORCES APPLIED TO THIS RIGID BODY
	/// </summary>
	/// <param name="i"> index of rigid body </param>
	/// <returns> sum of all forces applied to this rigid body</returns>
	vector<AppliedForce> resultingForce = vector<AppliedForce>();

	for (int j = 0; j < forcesOnAllRigidBodies.size(); j++) {
		AppliedForce curForce = forcesOnAllRigidBodies[j];
		if (curForce.rigidBodyIndex == i) {
			// if the force is applied to our object of interest,
			// we should accumulate it
			resultingForce.push_back(curForce);
		}

	}

	return resultingForce;
}

Mat4 RigidBodySystem::rigidBodyWorldPosition(int i)
{
	return rigidBodies[i].getObjectWorldPosition();
}
