#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Simulator.h"
#include "BaseObject.h"
#include "DrawingUtilitiesClass.h"


class RigidBody: public BaseObject {

	float _mass;
	Vec3 _size;
	Vec3 _torque;
	Vec3 _angularVel;
	Vec3 _angularMom;
	Quat _orientation;
	Mat4 _inertiaTensorInv;

public:
	RigidBody(Vec3 position, Vec3 size,
		int mass, bool fixed = false, bool userInteraction = false);

	void initInertiaInv();
	void resetForce();
	void applyForce(GamePhysics::Vec3 loc, GamePhysics::Vec3 force);

	float mass() const;
	Vec3 size() const;
	Vec3 torque() const;
	Vec3 angularVel() const;
	Vec3 angularMom() const;
	Quat angularVelQuat() const;
	Quat orientation() const;

	void size(Vec3 size);
	void torque(Vec3 torque);
	void angularVel(Vec3 angularVel);
	void angularMom(Vec3 angularMom);
	void orientation(Quat orientation);

	Mat4 scaleMatrix() const;
	Mat4 rotationMatrix() const;
	Mat4 translationMatrix() const;
	Mat4 obj2World() const;
	Mat4 inertiaTensorInv() const;

private:

};

inline float RigidBody::mass() const
{
	return _mass;
}

inline Vec3 RigidBody::size() const
{
	return _size;
}

inline Vec3 RigidBody::torque() const
{
	return _torque;
}

inline Vec3 RigidBody::angularVel() const
{
	return _angularVel;
}

inline Vec3 RigidBody::angularMom() const
{
	return _angularMom;
}

inline Quat RigidBody::angularVelQuat() const
{
	return Quat(_angularVel.x, _angularVel.y, _angularVel.z, 0);
}

inline Quat RigidBody::orientation() const
{
	return _orientation;
}

inline void RigidBody::size(Vec3 size)
{
	_size = size;
}

inline void RigidBody::torque(Vec3 torque)
{
	_torque = torque;
}

inline void RigidBody::angularVel(Vec3 angularVel)
{
	_angularVel = angularVel;
}

inline void RigidBody::angularMom(Vec3 angularMom)
{
	_angularMom = angularMom;
}

inline void RigidBody::orientation(Quat orientation)
{
	// this way orientation is always normalized :D
	_orientation = orientation.unit();
}

#endif