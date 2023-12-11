#pragma once
#include <stdio.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <random>

using namespace Eigen;
using namespace std;

#define PI 3.1415

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

	bool operator==(const myQuaternion& q) {
		return this->w == q.w && this->x == q.x && this->y == q.y == this->z == q.z;
	}

	float dot(myQuaternion q) {
		return this->x * q.x + this->y * q.y + this->z * q.z + this->w * q.w;
	}
};

myQuaternion slerp(myQuaternion q1, myQuaternion q2, float u) {
	if (q1 == q2) {
		return q1;
	}
	else
	{
		float omega = acos(q1.dot(q2));
		myQuaternion qOut = q1 * (sin((1 - u) * omega) / sin(omega)) + q2 * (sin(omega * u) / sin(omega));

		return qOut;
	}
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

class rigidBody {
public:
	myTransform transform;
	float m;
	Vector3f v;
	Vector3f a;
	float damping;
	float r;

	bool enableGravity=false;
	float dt = 0.016;

	//Init rigid body with random properties
	void init() {
		int min = -10, max = 10;
		random_device seed;
		ranlux48 engine(seed());
		uniform_int_distribution<> distrib(min, max);
		int random = distrib(engine);

		transform.location << distrib(engine), distrib(engine), distrib(engine);
		transform.rotation << 0, 0, 0;
		damping = (rand() % 100) / 100;
		m = distrib(engine)+20;
		v << 2*distrib(engine), 2*distrib(engine), 2*distrib(engine);
		r = 1+abs(distrib(engine))/5.0f;
	}

	//Reset acceleration
	void initAcc() {
		if (enableGravity) {
			a << 0, -9.8, 0;
		}
		else
		{
			a << 0, 0, 0;
		}
	}

	//Update velocity and position per frame
	void move() {
		v+= a * dt;
		transform.location += dt * v;
	}

	//Detect the relative position with ground and bounce if collided
	void bounce(float ground) {
		if (this->transform.location(1) < ground) {
			this->v(1) = abs(this->v(1));
		}
	}
};

void impact(rigidBody& m1, rigidBody& m2) {
	m1.v = sqrt(m1.damping) * (m1.m - m2.m) / (m1.m + m2.m) * m1.v + 2 * m2.m / (m1.m + m2.m) * m2.v;
	m2.v = sqrt(m2.damping) * (m2.m - m1.m) / (m1.m + m2.m) * m2.v + 2 * m1.m / (m1.m + m2.m) * m2.v;
}

void spiralField(rigidBody& obj,float strength,float radius) {
	double theta;

	float r = sqrt(obj.transform.location(0) * obj.transform.location(0) + obj.transform.location(2) * obj.transform.location(2));
	if (r < radius) {
		if (obj.transform.location(0) > 0) {
			theta = atan(obj.transform.location(2) / obj.transform.location(0));
		}
		else
		{
			theta = atan(obj.transform.location(2) / obj.transform.location(0)) + PI;
		}
		obj.a += Vector3f(-strength * sin(theta), 0, strength * cos(theta));
	}
}

void parallelField(rigidBody& obj, float strength, float radius=100) {
	float r = sqrt(obj.transform.location(0) * obj.transform.location(0) + obj.transform.location(2) * obj.transform.location(2));

	if (r < radius) {
		obj.a(1) += strength;
	}
}

void centeringField(rigidBody& obj, float strength, float radius) {
	double theta;
	float r = sqrt(obj.transform.location(0) * obj.transform.location(0) + obj.transform.location(2) * obj.transform.location(2));

	if (r > radius) {
		if (obj.transform.location(0) > 0) {
			theta = atan(obj.transform.location(2) / obj.transform.location(0));
		}
		else
		{
			theta = atan(obj.transform.location(2) / obj.transform.location(0)) + PI;
		}
		obj.a += Vector3f(-strength * cos(theta), 0, -strength * sin(theta));
	}
}

void universalGravitation(vector<rigidBody>& objs, float cGrav, float cRep) {
	for (int i = 0; i < objs.size(); i++) {
		for (int j = 0; j < objs.size(); j++) {
			if (i != j) {
				float r = (objs[i].transform.location - objs[j].transform.location).norm();
				objs[i].a += (cGrav * objs[j].m / pow(r, 4) - cRep * objs[j].m / pow(r, 6)) * (objs[j].transform.location - objs[i].transform.location).normalized();
			}
		}
	}
}

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
		1 - 2 * y*y-2*z*z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y, 0,
		2 * x * y + 2 * w * z, 1-2*x*x-2*z*z, 2 * y * z - 2 * w * x, 0,
		2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1-2*x*x-2*y*y, 0,
		0, 0, 0, 1;

	return mat;
}

Matrix4f translate(float x, float y, float z) {
	Matrix4f matTrans;
	matTrans <<
		1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1;

	return matTrans;
}

myQuaternion angleToQuat(float psi, float theta, float phi) {
	myQuaternion qx(cos((psi / 2) * PI / 180), sin((psi / 2) * PI / 180), 0, 0);
	myQuaternion qy(cos((theta / 2) * PI / 180), 0, sin((theta / 2) * PI / 180), 0);
	myQuaternion qz(cos((phi / 2) * PI / 180), 0, 0, sin((phi / 2) * PI / 180));
	myQuaternion qOut = qz * (qy * qx);
	
	return qOut;
}

Matrix4f modelMat(myTransform trans) {
	Matrix4f matTrans;
	Matrix4f matRotate;
	Matrix4f matModel;
	GLfloat mat[16]{};

	matTrans = translate(trans.location(0), trans.location(1), trans.location(2));
	matRotate = quatRotate(trans.quat.w, trans.quat.x, trans.quat.y, trans.quat.z);
	matModel = matTrans* matRotate;

	return matModel;
}

//Accept a set of k-frames, generate the sequence with 24 in-betweens based on Catmall spline
Sequence Catmall_Rom(Sequence seqInput, bool isQuat) {
	int frameRate = 24;
	MatrixXf T(1, 4);
	MatrixXf M(4, 4);
	Sequence seqOutput;
	MatrixXf Glocation(4, 3);
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

		Vector3f dirBefore = seqInput.sequence[i + 1].location - seqInput.sequence[i].location;
		Vector3f dirAfter = seqInput.sequence[i + 2].location - seqInput.sequence[i+1].location;

		/*
		//transfer fixed angle to quaternion
		if (!isQuat) {
			q1 = angleToQuat(seqInput.sequence[i].rotation(0), seqInput.sequence[i].rotation(1), seqInput.sequence[i].rotation(2));
			q2 = angleToQuat(seqInput.sequence[i + 1].rotation(0), seqInput.sequence[i + 1].rotation(1), seqInput.sequence[i + 1].rotation(2));
		}
		else{
			q1 = seqInput.sequence[i].quat;
			q2 = seqInput.sequence[i + 1].quat;
		}
		*/

		q1 = angleToQuat(0, atan(dirBefore(2) / dirAfter(0)) * 180 / PI, 0);
		q2 = angleToQuat(0, atan(dirAfter(2) / dirAfter(0)) * 180 / PI, 0);

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

//Accept a set of k-frames, generate the sequence with 24 in-betweens based on B-spline
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

		/*
		//transfer fixed angle to quaternion
		if (!isQuat) {
			q1 = angleToQuat(seqInput.sequence[i].rotation(0), seqInput.sequence[i].rotation(1), seqInput.sequence[i].rotation(2));
			q2 = angleToQuat(seqInput.sequence[i + 1].rotation(0), seqInput.sequence[i + 1].rotation(1), seqInput.sequence[i + 1].rotation(2));
		}
		else {
			q1 = seqInput.sequence[i].quat;
			q2 = seqInput.sequence[i + 1].quat;
		}
		*/

		

		//update t with framerate
		for (float t = 0; t <= 1; t += 1.0 / frameRate) {
			myTransform frame;
			T << pow(t, 3), pow(t, 2), pow(t, 1), pow(t, 0);

			frame.location = T * M * Glocation;
			//frame.quat = slerp(q1, q2, t);
			seqOutput.sequence.push_back(frame);
		}
	}
	for (int i = 0; i < seqOutput.sequence.size()-1; i++) {		
		seqOutput.sequence[i].rotation = seqOutput.sequence[i + 1].location - seqOutput.sequence[i].location;
		seqOutput.sequence[i].quat = angleToQuat(seqOutput.sequence[i].rotation(0), seqOutput.sequence[i].rotation(1), seqOutput.sequence[i].rotation(2));
	}
	return seqOutput;
}

//
void attract(rigidBody& obj1, rigidBody& obj2, float strength) {
	obj1.a += strength * (obj2.transform.location - obj1.transform.location).normalized();
}

void attractR(rigidBody& obj1, rigidBody& obj2, float strength) {
	float r = (obj2.transform.location - obj1.transform.location).norm();
	obj1.a += strength * (1 / r) * (obj2.transform.location - obj1.transform.location).normalized();
}

//
void herb(vector<rigidBody>& objs, rigidBody target, rigidBody predator) {
	float dist = 100;
	int lead={};

	for (int i = 0; i < objs.size(); i++) {
		float r = abs((objs[i].transform.location - target.transform.location).norm());
		if (r < dist) {
			dist = r;
			lead = i;
		}
	}
	attract(objs[lead], target, 30);

	for (int i = 0; i < objs.size(); i++) {
		if (i != lead) {
			attract(objs[i], objs[lead], 25);
		}
	}

	for (int i = 0; i < objs.size(); i++) {
		for (int j = 0; j < objs.size(); j++) {
			if (i != j) {
				attractR(objs[i], objs[j], -0.1);
			}
		}
	}
}