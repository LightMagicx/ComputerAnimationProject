#pragma once
#include <stdio.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace Eigen;
using namespace std;

class myQuaternion {
public:
	float w, x, y, z;

	myQuaternion(float w, float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}
	myQuaternion() { this->x = 0, this->y = 0, this->z = 0, this->w = 0; }

	myQuaternion operator*(const myQuaternion& q) {
		myQuaternion result;
		Matrix<float, 1, 3> v1, v2, v3;
		v1 << this->x, this->y, this->z;
		v2 << q.x, q.y, q.z;

		result.w = this->w * q.w - v1.dot(v2);
		v3 = this->w * v2 + q.w * v1 + v1.cross(v2);
		result.x = v3(0);
		result.y = v3(1);
		result.z = v3(2);

		return result;
	}

	myQuaternion operator*(const float a) {
		myQuaternion result(a * this->w, a * this->x, a * this->y, a * this->z);

		return result;
	}

	myQuaternion operator+(const myQuaternion& q) {
		myQuaternion result(q.w + this->w, q.x + this->x, q.y + this->y, q.z + this->z);

		return result;
	}

	float dot(myQuaternion q) {
		return this->x * q.x + this->y * q.y + this->z * q.z + this->w * q.w;
	}
};

myQuaternion slerp(myQuaternion q1, myQuaternion q2, float u) {
	float omega = acos(q1.dot(q2));
	myQuaternion qOut = q1 * (sin((1 - u) * omega) / sin(omega)) + q2 * (sin(omega * u) / sin(omega));

	return qOut;
}

class myTransform {
public:
	Matrix<float, 1, 3> location;
	Matrix<float, 1, 3> rotation;
	myQuaternion quat;
};

class Sequence {
public:
	vector<myTransform> sequence;
};

//Rotation matrix of X-axis
Matrix4f rotateX(float psi) {
	Matrix4f matX;
	matX <<
		1, 0, 0, 0,
		0, cos(psi), -sin(psi), 0,
		0, sin(psi), cos(psi), 0,
		0, 0, 0, 1;

	return matX;
}

///Rotation matrix of Y-axis
Matrix4f rotateY(float theta) {
	Matrix4f matY;
	matY <<
		cos(theta), 0, sin(theta), 0,
		0, 1, 0, 0,
		-sin(theta), 0, cos(theta), 0,
		0, 0, 0, 1;

	return matY;
}

//Rotation matrix of Z-axis
Matrix4f rotateZ(float phi) {
	Matrix4f matZ;
	matZ <<
		cos(phi), -sin(phi), 0, 0,
		sin(phi), cos(phi), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return matZ;
}

//Fixed angle rotation 4x4matrix with X*Y*Z*A order
Matrix4f fixedAngleRotate(float psi, float theta, float phi) {
	
	return rotateX(psi) * rotateY(theta) * rotateZ(phi);
}

//Quaternion rotation 3x3matrix
Matrix4f quatRotate(float w, float x, float y, float z) {
	Matrix4f mat;
	mat <<
		(1 - 2 * pow(y, 2) - 2 * pow(z, 2)), (2 * x * y - 2 * w * z), (2 * x * z + 2 * w * y),
		(2 * x * y + 2 * w * z), (1 - 2 * pow(x, 2) * pow(y, 2)), (2 * y * z - 2 * w * x),
		(2 * x * z - 2 * w * y), (2 * y * z + 2 * w * x), (1 - 2 * pow(x, 2) - 2 * pow(y, 2));

	return mat;
}

myQuaternion angleToQuat(float psi, float theta, float phi) {
	myQuaternion qx(cos(psi / 2), sin(psi / 2), 0, 0);
	myQuaternion qy(cos(theta / 2), 0, sin(theta / 2), 0);
	myQuaternion qz(cos(phi / 2), 0, 0, sin(phi / 2));
	myQuaternion qOut = qz * (qy * qx);

	return qOut;
}

//Accept a set of k-frames, generate the sequence with 24 in-betweens based on Catmall spline
Sequence Catmall_Rom(Sequence seqInput, bool isQuat) {
	int frameRate = 24;
	MatrixXf T(1, 4);
	MatrixXf M(4, 4);
	Sequence seqOutput;
	MatrixXf Glocation(4, 3);
	/*
	MatrixXf Grotation(4, 3);
	*/
	float a = 0.5;

	M <<
		(-a), (2 - a), (a - 2), (a),
		(2 * a), (a - 3), (3 - 2 * a), (-a),
		(-a), 0, (a), 0,
		0, 1, 0, 0;

	for (int i = 1; i <= seqInput.sequence.size() - 3; i++) {
		myQuaternion q1, q2;

		Glocation.row(0) = seqInput.sequence[i - 1].location;
		Glocation.row(1) = seqInput.sequence[i].location;
		Glocation.row(2) = seqInput.sequence[i + 1].location;
		Glocation.row(3) = seqInput.sequence[i + 2].location;

		//transfer fixed angle to quaternion
		if (!isQuat) {
			q1 = angleToQuat(seqInput.sequence[i].rotation(0), seqInput.sequence[i].rotation(1), seqInput.sequence[i].rotation(2));
			q2 = angleToQuat(seqInput.sequence[i + 1].rotation(0), seqInput.sequence[i + 1].rotation(1), seqInput.sequence[i + 1].rotation(2));
		}
		else{
			q1 = seqInput.sequence[i].quat;
			q2 = seqInput.sequence[i + 1].quat;
		}

		//update t with framerate
		for (float t = 0; t <= 1; t += 1.0 / frameRate) {
			T << pow(t, 3), pow(t, 2), pow(t, 1), pow(t, 0);
			myTransform frame;

			frame.location = T * M * Glocation;
			/*
			frame.rotation = T * M * Grotation;
			*/
			
			frame.quat = slerp(q1, q2, t);
			seqOutput.sequence.push_back(frame);

		}
	}

	return seqOutput;
}

//
Sequence Bspline(Sequence seqInput, bool isQuat) {
	int frameRate = 24;
	MatrixXf T(1, 4);
	MatrixXf M(4, 4);
	Sequence seqOutput;
	MatrixXf Glocation(4, 3);

	M <<
		-1, 3, -3, 1,
		3, -6, 3, 0,
		-3, 0, 3, 0,
		1, 4, 1, 0;
	M = M / 6;

	for (int i = 1; i <= seqInput.sequence.size() - 3; i++) {
		myQuaternion q1, q2;

		Glocation.row(0) = seqInput.sequence[i - 1].location;
		Glocation.row(1) = seqInput.sequence[i].location;
		Glocation.row(2) = seqInput.sequence[i + 1].location;
		Glocation.row(3) = seqInput.sequence[i + 2].location;

		//transfer fixed angle to quaternion
		if (!isQuat) {
			q1 = angleToQuat(seqInput.sequence[i].rotation(0), seqInput.sequence[i].rotation(1), seqInput.sequence[i].rotation(2));
			q2 = angleToQuat(seqInput.sequence[i + 1].rotation(0), seqInput.sequence[i + 1].rotation(1), seqInput.sequence[i + 1].rotation(2));
		}
		else {
			q1 = seqInput.sequence[i].quat;
			q2 = seqInput.sequence[i + 1].quat;
		}

		//update t with framerate
		for (float t = 0; t <= 1; t += 1.0 / frameRate) {
			myTransform frame;
			T << pow(t, 3), pow(t, 2), pow(t, 1), pow(t, 0);

			frame.location = T * M * Glocation;

			frame.quat = slerp(q1, q2, t);
			seqOutput.sequence.push_back(frame);
		}
	}

	return seqOutput;
}
