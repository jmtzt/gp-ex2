#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
	: 
	_gravityEnabled(false),
	_gravity(Vec3(0, -10, 0)),
	_collisionBounciness(1)
{}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_pRigidBodySystem.clear();
	_gravityEnabled = false;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		DUC->drawRigidBody(m_pRigidBodySystem[i].obj2World());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	reset();
	m_iTestCase = testCase;
	Vec3 xw, vw;
	switch (m_iTestCase)
	{
	case 0:
		cout << "--Demo1-- timeStep = 2, single rigid box" << endl;
		addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.f, 0.6f, 0.5f), 2.f);
		setOrientationOf(0, Quat(Vec3(0.f, 0.f, 1.f), (float)0.5f*M_PI));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.f, 1.f, 0.f));

		simulateTimestep(2);

		xw = Vec3(-0.3f, -0.5f, -0.25f) - getPositionOfRigidBody(0);
		vw = getLinearVelocityOfRigidBody(0) + cross(getAngularVelocityOfRigidBody(0), xw);
		
		cout << "Angular Velocity : " << getAngularVelocityOfRigidBody(0) << endl;
		cout << "Position_w: " << xw << endl;
		cout << "Velocity_w: " << vw << endl;

		break;
	case 1:
		cout << "--Demo2-- timeStep = 0.01, single rigid box" << endl;
		addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.f, 0.6f, 0.5f), 2.f);
		setOrientationOf(0, Quat(Vec3(0.f, 0.f, 1.f), (float)0.5f * M_PI));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.f, 1.f, 0.f));

		simulateTimestep(0.01);

		xw = Vec3(-0.3f, -0.5f, -0.25f) - getPositionOfRigidBody(0);
		vw = getLinearVelocityOfRigidBody(0) + cross(getAngularVelocityOfRigidBody(0), xw);

		cout << "Angular Velocity : " << getAngularVelocityOfRigidBody(0) << endl;
		cout << "Position_w: " << xw << endl;
		cout << "Velocity_w: " << vw << endl;
		break;
	case 2:
		cout << "--Demo3-- two rigid boxes w/ collision" << endl;
		addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);

		addRigidBody(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)0.25f *(M_PI)));
		setVelocityOf(1, Vec3(0.0f, -0.1f, 0.05f));
		break;
	case 3:
		cout << "--Demo4-- 4 rigid boxes w/ collision + interaction\n" << endl;
		addRigidBody(Vec3(0.75f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 100.0f);
		setVelocityOf(0, Vec3(+0.5f, 0.0f, 0.0f));

		addRigidBody(Vec3(1.5f, 1.25f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 100.0f);
		setVelocityOf(1, Vec3(-0.1f, -0.3f, 0.0f));

		addRigidBody(Vec3(2.0f, +0.55f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 100.0f);
		setVelocityOf(2, Vec3(-0.40f, -0.0f, 0.0f));

		addRigidBody(Vec3(2.25f, +1.5f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 100.0f);
		setVelocityOf(3, Vec3(-0.20f, -0.4f, 0.0f));
		_gravityEnabled = true;
	default: break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0) {
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		float scale = 0.01f;
		for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
			m_pRigidBodySystem[i].applyForce(m_pRigidBodySystem[i].position(), inputView*scale);
		}
	}
	// also verify gravity
	if (_gravityEnabled) {
		for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
			m_pRigidBodySystem[i].applyForce(m_pRigidBodySystem[i].position(), m_pRigidBodySystem[i].mass() * _gravity);
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	checkCollision();
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		if (m_pRigidBodySystem[i].fixed()) {
			continue;
		}

		// euler step for pos and vel
		m_pRigidBodySystem[i].position(m_pRigidBodySystem[i].position() 
									   + timeStep * m_pRigidBodySystem[i].velocity());
		m_pRigidBodySystem[i].velocity(m_pRigidBodySystem[i].velocity()
									   + (timeStep * m_pRigidBodySystem[i].force() / m_pRigidBodySystem[i].mass()));

		// update orientation quaternion
		m_pRigidBodySystem[i].orientation(m_pRigidBodySystem[i].orientation()
										  + (timeStep / 2.f) * m_pRigidBodySystem[i].angularVelQuat() * m_pRigidBodySystem[i].orientation());
		// update angular momentum
		m_pRigidBodySystem[i].angularMom(m_pRigidBodySystem[i].angularMom() + timeStep * m_pRigidBodySystem[i].torque());

		// update angular velocity
		m_pRigidBodySystem[i].angularVel(m_pRigidBodySystem[i].inertiaTensorInv() * m_pRigidBodySystem[i].angularMom());

		// clear this objects forces
		m_pRigidBodySystem[i].resetForce();
	}	
	checkFloor();
}

void RigidBodySystemSimulator::checkCollision()
{
	// determine collision relative position for each rigidbody
	//Once you find a collision, your collision response function should calculate :
	// vrel, the relative velocity between Aand B at the collision point(in world space).
	// -> its the velocity of point A minus the vel of point B wrt to the relative collision positions
	// If vrel· n > 0, this indicates that the bodies are separating.
	// Otherwise continue to calculate the impulse J, and apply it to both bodies.
	for (int i = 0; i < m_pRigidBodySystem.size(); ++i) {
		RigidBody bodyA = m_pRigidBodySystem[i];
		for (int j = i + 1; j < m_pRigidBodySystem.size(); ++j) {
			RigidBody bodyB = m_pRigidBodySystem[j];
			CollisionInfo info = checkCollisionSAT(bodyA.obj2World(), bodyB.obj2World());
			if (info.isValid) {
				
				// info.collisionPointWorld is the position of the collision point(one vertex of B) in world space
				// determine relative positions
				Vec3 colPosA = info.collisionPointWorld - bodyA.position();
				Vec3 colPosB = info.collisionPointWorld - bodyB.position();
			
				// determine velocities of collision points
				// vw = vi + w x x_i
				Vec3 velA = bodyA.velocity() + cross(bodyA.angularVel(), colPosA);
				Vec3 velB = bodyB.velocity() + cross(bodyB.angularVel(), colPosB);
				Vec3 velRelative = velA - velB;

				// info.normalWorld is the normalized direction of the impulse from B -> A
				// check velRelative dot info.normalWorld < 0, then the bodies are colliding
				// so we need to calculate the impulse J
				// J = -(1+c)vrel*n / (1/massA + 1/massB + (cross(Ia-1 * cross(posA, n), posA) + Ib-1 * cross(posB, n) cross positionB) dot n
				// OBS:  v = Mat.transformVector(v). This will do the calculation of v= v * Mat
				if (dot(velRelative, info.normalWorld) < 0) {
					// 
					
					Vec3 crossA = cross(bodyA.inertiaTensorInv().transformVector(cross(colPosA, info.normalWorld)), colPosA);
					Vec3 crossB = cross(bodyB.inertiaTensorInv().transformVector(cross(colPosB, info.normalWorld)), colPosB);
					float invMassA = 1.0f / bodyA.mass();
					float invMassB = 1.0f / bodyB.mass();
					float J = -(1 + _collisionBounciness) * dot(velRelative, info.normalWorld)
						/ ( invMassA + invMassB + dot(crossA + crossB, info.normalWorld));
					// apply impulse
					// remember to change sign
					bodyA.velocity(bodyA.velocity() + J * info.normalWorld / bodyA.mass()); 
					bodyB.velocity(bodyB.velocity() - J * info.normalWorld / bodyB.mass());
					bodyA.angularMom(bodyA.angularMom() + cross(colPosA, J * info.normalWorld));
					bodyB.angularMom(bodyB.angularMom() - cross(colPosB, J * info.normalWorld));

					m_pRigidBodySystem[i] = bodyA;
					m_pRigidBodySystem[j] = bodyB;
				}				
			}
		}
	}

}

void RigidBodySystemSimulator::checkFloor()
{
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		Vec3 pos = m_pRigidBodySystem[i].position();
		Vec3 vel = m_pRigidBodySystem[i].velocity();
		if (pos.Y < - 0.5f && vel.Y < 0) {
			Vec3 new_vel = Vec3(vel.X, -0.9 * vel.Y, vel.Z);
			m_pRigidBodySystem[i].velocity(new_vel);
		}
	}

}


void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_pRigidBodySystem.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_pRigidBodySystem[i].position();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem[i].velocity();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem[i].angularVel();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_pRigidBodySystem[i].applyForce(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_pRigidBodySystem.push_back(RigidBody(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_pRigidBodySystem[i].orientation(orientation);
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_pRigidBodySystem[i].velocity(velocity);
}

