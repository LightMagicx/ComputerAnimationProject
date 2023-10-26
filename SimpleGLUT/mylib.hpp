#pragma once
#include <stdio.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace Eigen;
using namespace std;

class myTransform {
public:
	Vector3f location;
	Vector3f rotation;
};

class Sequence {
public:
	vector<myTransform> sequence;
};

/*
* Sequence animator(Sequence seq, float time) {
	int frame_num = time * 1000 / seq.sequence.size();

	

}
*/


//Accept a set of k-frames, generate the sequence with 24 in-betweens based on Catmall spline
Sequence Catmall_Rom(Sequence seqInput) {
	MatrixXf T(1,4);
	MatrixXf M(4,4);
	Sequence seqOutput;
	MatrixXf Glocation(4,3);
	MatrixXf Grotation(4,3);
	float t = 0, a = 0.5;

	M <<
		(-a), (2 - a), (a - 2), (a),
		(2 * a), (a - 3), (3 - 2 * a), (-a), 
		(-a), 0, (a), 0, 
		0, 1, 0, 0;

	for (int i = 1; i <= seqInput.sequence.size() - 3; i++) {
		
		Glocation.row(0) = seqInput.sequence[i - 1].location;
		Glocation.row(1) = seqInput.sequence[i].location;
		Glocation.row(2) = seqInput.sequence[i + 1].location;
		Glocation.row(3) = seqInput.sequence[i + 2].location;

		Grotation.row(0) = seqInput.sequence[i - 1].rotation;
		Grotation.row(1) = seqInput.sequence[i].rotation;
		Grotation.row(2) = seqInput.sequence[i + 1].rotation;
		Grotation.row(3) = seqInput.sequence[i + 2].rotation;
		
		//update t with 24fps
		for (; t <= 1; t += 1.0 / 24) {
			T << pow(t, 3), pow(t, 2), pow(t, 1), pow(t, 0);
			myTransform frame;

			cout << T * M * Glocation << endl;

			
			/*
			frame.location << T * M * Glocation;
			frame.rotation << T * M * Grotation;
			seqOutput.sequence.push_back(frame);
			*/
			
		}
	}
	
	return seqOutput;
}

/*
* Sequence Bspline(myTransform kf0, myTransform kf1, myTransform kf2, myTransform kf3) {

}
*/


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