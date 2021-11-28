#ifndef OBJECT_H
#define OBJECT_H

#include "Simulator.h"

class BaseObject
{
	bool _userInteraction;
	bool _fixed;
	std::string _tag;

	Vec3 _position;
	Vec3 _velocity;
	Vec3 _force;

public:

	BaseObject(Vec3 position, bool fixed = false, bool userInteraction = true);

	// the behaviour of object can be overridden in derived classes, hence we need to set virtual ~Object()
	virtual ~BaseObject();
	// Getters
	bool fixed() const;
	bool userInteraction() const;
	Vec3 position() const;
	Vec3 velocity() const;
	Vec3 force() const;
	std::string tag() const;

	// Setters
	void position(Vec3 position);
	void velocity(Vec3 velocity);
	void addForce(Vec3 f);
	void tag(std::string tag);
	void fixed(bool fixed);
	virtual void resetForce();

};

inline BaseObject::BaseObject(Vec3 position, bool fixed, bool userInteraction)
	: _position(position),
	_fixed(fixed),
	_userInteraction(userInteraction),
	_velocity(Vec3(0,0,0))
{}

inline BaseObject::~BaseObject()
{}

inline bool BaseObject::fixed() const
{
	return _fixed;
}

inline bool BaseObject::userInteraction() const
{
	return _userInteraction;
}

inline Vec3 BaseObject::position() const
{
	return _position;
}

inline Vec3 BaseObject::velocity() const
{
	return _velocity;
}

inline Vec3 BaseObject::force() const
{
	return _force;
}

inline std::string BaseObject::tag() const
{
	return _tag;
}

inline void BaseObject::position(Vec3 position)
{
	_position = position;
}

inline void BaseObject::velocity(Vec3 velocity)
{
	_velocity = velocity;
}

inline void BaseObject::addForce(Vec3 f)
{
	_force += f;
}

inline void BaseObject::tag(std::string tag)
{
	_tag = tag;
}

inline void BaseObject::fixed(bool fixed)
{
	_fixed = fixed;
}

inline void BaseObject::resetForce()
{
	_force = Vec3(0, 0, 0);
}

#endif