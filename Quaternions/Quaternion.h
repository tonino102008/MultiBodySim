#ifndef MULTIBODYSIM_QUATERNIONS_QUATERNION_H_
#define MULTIBODYSIM_QUATERNIONS_QUATERNION_H_

#include <vector>

class Quaternion {

public:

	Quaternion();

	Quaternion(const double s, const std::vector<double> v);

	double getScalar() const;

	std::vector<double> getVector() const;

	Quaternion operator+(const Quaternion& q) const;

	Quaternion operator*(const Quaternion& q) const;

	Quaternion conj() const;

	double norm() const;

private:

	double scalar_;

	std::vector<double> vector_;

};

#endif // MULTIBODYSIM_QUATERNIONS_QUATERNION_H_