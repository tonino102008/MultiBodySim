#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_QUATERNIONS_QUATERNION_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_QUATERNIONS_QUATERNION_H_

#include <vector>
#include <cmath>
#include <ostream>
#include "RigidBody/VectorN/VectorN.h"

class Quaternion {

public:

	Quaternion();

	Quaternion(const double s, const VectorN v);

	double getScalar() const;

	VectorN getVector() const;

	Quaternion operator+(const Quaternion& q) const;

	Quaternion operator*(const Quaternion& q) const;

	friend std::ostream& operator<<(std::ostream& out, const Quaternion& q);

	Quaternion conj() const;

	double norm() const;

private:

	double scalar_;

	VectorN vector_;

};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_QUATERNIONS_QUATERNION_H_