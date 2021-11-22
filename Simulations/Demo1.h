#pragma once

#include "RigidBody.h"
#include "RigidBodySystem.h"
#include "DrawingUtilitiesClass.h"

class Demo1
{
public:
	Demo1(){
		m_pRigidBodySystem.addRigidBody(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.6, 0.5), 2.f);
		reset();
	}

	void update(float timeStep) {
		
	}

	void test() {
		RigidBody rigidBody = m_pRigidBodySystem.rigidBodies[0];
		vector<vector<float>> inertia = rigidBody.inertiaTensor;

		float timeStep = 2;

		cout << "i[0][0]: " << inertia[0][0] << endl;
		cout << "i[1][1]: " << inertia[1][1] << endl;
		cout << "i[2][2]: " << inertia[2][2] << endl;
		cout << "i-1[0][0]: " << 1.0f/inertia[0][0] << endl;
		cout << "i-1[1][1]: " << 1.0f/inertia[1][1] << endl;
		cout << "i-1[2][2]: " << 1.0f/inertia[2][2] << endl;

		matrix4x4<double> I_not = (matrix4x4<double>) rigidBody.currentInverseInertia();
		vector3Dim<double> L = vector3Dim<double>(0, 0, 0);
		
		vector3Dim<double> w = I_not * L;

		vector3Dim<double> x = vector3Dim<double>(0.3, 0.5, 0.25);
		
		vector3Dim<double> f = vector3Dim<double>(1, 1, 0);

		vector3Dim<double> q = cross(x, f);

		vector3Dim<double> x_cm = vector3Dim<double>(0, 0, 0);
		vector3Dim<double> v_cm = vector3Dim<double>(0, 0, 0);

		x_cm += timeStep * v_cm;
		v_cm += timeStep * (f / rigidBody.mass);

		cout << "xcm" << x_cm << endl;
		cout << "vcm" << v_cm << endl;

		rigidBody.orientation += (timeStep / 2) * Quat(w, 0) * rigidBody.orientation;

		L += timeStep * q;

		matrix4x4<double> I_inv = (matrix4x4<double>) rigidBody.currentInverseInertia();

		w = I_inv * L;

		cout << "w" << w << "\n";
		vector3Dim<double> x_i = vector3Dim<double>(-0.3, -0.5, -0.25);
		vector3Dim<double> v_w = v_cm + cross(w, x_i);

		cout << "xi" << x_i << endl;
		cout << "orientation" << rigidBody.orientation.getRotMat() << endl;
		matrix4x4<double> rotr = rigidBody.orientation.getRotMat();
		rotr.transpose();
		vector3Dim<double> x_w = x_cm + rotr * x_i;
		////rigidBody.orientation.getRotMat().transpose();
		cout << "x_w" << x_w << "\n";
		cout << "v_w" << v_w << "\n";
	

	}

	void draw(DrawingUtilitiesClass* DUC) {
		
	}

private:
	void reset() {
		m_pRigidBodySystem.rigidBodies[0].position = Vec3(0.0, 0.0, 0.0);
		m_pRigidBodySystem.rigidBodies[0].size = Vec3(1.0, 0.6, 0.5);
		m_pRigidBodySystem.rigidBodies[0].mass = 2;
		m_pRigidBodySystem.setOrientationOf(0, Quat(Vec3(0.0, 0.0, 1.0), M_PI * 0.5f));
	}

	void print() {
		cout << "----------------------" << endl;

		cout << "----------------------" << endl << endl;
	}

	void _update(float timeStep, int integrator, bool addGravity = false) {
		
	}

	RigidBodySystem m_pRigidBodySystem;
};
