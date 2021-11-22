#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Simulator.h"

class RigidBody {
public:
	// Constructor
	RigidBody(Vec3 position, Vec3 size, float mass);
	matrix4x4<double> getObjectWorldPosition();
	matrix4x4<double> currentInverseInertia();
	void calculateInertia();
	vector<vector<float>> inertiaTensor;

	// Attributes
	Vec3 position;
	Vec3 size;
	float mass;
	Quat orientation;
	Vec3 angularVel;
	Vec3 linearVel;
	Vec3 torque;

private:

};

#endif