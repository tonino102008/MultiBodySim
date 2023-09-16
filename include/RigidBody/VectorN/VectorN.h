#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_VECTORN_VECTORN_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_VECTORN_VECTORN_H_

#include <vector>
#include <cmath>
#include <ostream>

class VectorN {

public:

	VectorN(const int size);

	int getSize() const;

	double& operator[](const int i);

	double operator[](const int i) const;

	VectorN& operator=(const VectorN& v);

	VectorN operator+(const VectorN& v) const;

	VectorN operator-(const VectorN& v) const;

	VectorN operator*(const VectorN& v) const;

	VectorN operator*(const double d) const;

	double dot(const VectorN& v) const;

	VectorN cross(const VectorN& v) const;

	double norm() const;

private:

	const int ksize_;

	std::vector<double> vector_;

};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_VECTORN_VECTORN_H_