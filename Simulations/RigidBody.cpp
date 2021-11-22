#include "RigidBody.h"

vector<vector<float>> convertTo3D(Mat4 m) {
	vector<vector<float>> res = vector<vector<float>>();

	for (int i = 0; i < 3; i++) {
		vector<float> curRow = vector<float>();
		for (int j = 0; j < 3; j++) {
			curRow.push_back(m.value[i][j]);
		}
		res.push_back(curRow);
	}

	return res;
}

RigidBody::RigidBody(Vec3 position, Vec3 size, float mass)
{
	// TODO
	// constructor for rigidbody
	this->position = position;
	this->size = size;
	this->mass = mass;
	// TODO - rotate 90 around z-axis
	this->orientation = Quat();
	this->angularVel = Vec3();
	this->linearVel = Vec3();
	this->torque = Vec3();

	// initialize inertia tensor
	calculateInertia();
}

matrix4x4<double> RigidBody::getObjectWorldPosition()
{
	// TODO
	// convert object position to world position
	// using rotation matrices
	
	return matrix4x4<double>();
}

matrix4x4<double> RigidBody::currentInverseInertia()
{
	// TODO
	// get inverse inertia tensor
	// Rotr * Io-1 * Rotr
	matrix4x4<double> rotr_t = orientation.getRotMat();

	//vector<vector<float>> rotr3D_t = convertTo3D(rotr);
	//rotr.transpose();
	//vector<vector<float>> rotr3D = convertTo3D(rotr);


	matrix4x4<double> Io = matrix4x4<double>(inertiaTensor[0][0], inertiaTensor[0][1], inertiaTensor[0][2], inertiaTensor[0][3],
		inertiaTensor[1][0], inertiaTensor[1][1], inertiaTensor[1][2], inertiaTensor[1][3],
		inertiaTensor[2][0], inertiaTensor[2][1], inertiaTensor[2][2], inertiaTensor[2][3],
		inertiaTensor[3][0], inertiaTensor[3][1], inertiaTensor[3][2], inertiaTensor[3][3]);

	matrix4x4<double> Io_inv = Io.inverse();

	matrix4x4<double> tmp = Io_inv * rotr_t;
	
	//cout << "rotr_transpose\n" << rotr_t;
	
	rotr_t.transpose();

	//cout << "rotr\n" << rotr_t;

	return rotr_t * tmp;
}

void RigidBody::calculateInertia()
{
	
	for (int i = 0; i < 4; i++) {
		inertiaTensor.push_back(vector<float>());
		for (int j = 0; j < 4; j++){
			inertiaTensor[i].push_back(0);
		}
	}
	// according to the cuboid inertia formula, we have in the main diagonal
	// the following values
	inertiaTensor[0][0] = (1 / 12.0) * mass * (pow(size.y, 2) + pow(size.z, 2));
	inertiaTensor[1][1] = (1 / 12.0) * mass * (pow(size.x, 2) + pow(size.z, 2));
	inertiaTensor[2][2] = (1 / 12.0) * mass * (pow(size.x, 2) + pow(size.y, 2));
	inertiaTensor[3][3] = 1;

}

