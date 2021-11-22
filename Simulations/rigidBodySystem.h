#ifndef RIGIDBODYSYSTEM_h
#define RIGIDBODYSYSTEM_h

#include "rigidBody.h"

struct AppliedForce {
	Vec3 position;
	Vec3 force;
	int rigidBodyIndex;
};

class RigidBodySystem {
public:
	// Construtors
	RigidBodySystem();
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	void resetForces();
	vector<AppliedForce> allForcesOnRigidBody(int i);
	Mat4 rigidBodyWorldPosition(int i);
	// Attributes
	vector<RigidBody> rigidBodies;

private:
	vector<AppliedForce> forcesOnAllRigidBodies;
};

#endif