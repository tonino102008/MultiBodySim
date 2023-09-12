#ifndef Quaternion_h
#define Quaternion_h

#include <vector>

class Quaternion {

private:

	double scalar;

	std::vector<double> vector;

public:

	Quaternion();

	Quaternion(const double s, const std::vector<double> v);

	double getScalar() const;

	std::vector<double> getVector() const;

	Quaternion operator+(const Quaternion& q) const;

	Quaternion operator*(const Quaternion& q) const;

	Quaternion conj() const;

	double norm() const;

};

#endif