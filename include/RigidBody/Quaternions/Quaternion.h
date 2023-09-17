#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_QUATERNIONS_QUATERNION_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_QUATERNIONS_QUATERNION_H_

#include "RigidBody/MatrixN/MatrixN.h"

#include <cmath>
#include <ostream>

class Quaternion {

public:

	Quaternion();

	Quaternion(const double s, const MatrixN& m);

	double getScalar() const;

	MatrixN getVector() const;

	Quaternion operator+(const Quaternion& q) const;

	Quaternion operator*(const Quaternion& q) const;

	friend std::ostream& operator<<(std::ostream& out, const Quaternion& q);

	Quaternion conj() const;

	double norm() const;

private:

	double scalar_;

	MatrixN vector_;

};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_QUATERNIONS_QUATERNION_H_