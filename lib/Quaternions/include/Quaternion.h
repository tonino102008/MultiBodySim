#ifndef MULTIBODYSIM_LIB_QUATERNIONS_QUATERNION_H_
#define MULTIBODYSIM_LIB_QUATERNIONS_QUATERNION_H_

#include "Eigen/Dense"

#include <cmath>
#include <ostream>

class Quaternion {

public:

	Quaternion();

	Quaternion(const double s, const Eigen::Vector3d& m);

	Eigen::VectorXd getQuaternion() const;

	double getScalar() const;

	Eigen::Vector3d getVector() const;

	double qx() const;

	double qy() const;

	double qz() const;

	Quaternion operator+(const Quaternion& q) const;

	Quaternion operator*(const Quaternion& q) const;

	friend std::ostream& operator<<(std::ostream& out, const Quaternion& q);

	Quaternion conj() const;

	double norm() const;

	Eigen::Vector3d rotateVecG(const Eigen::Vector3d& m) const;

	Eigen::Vector3d rotateVecL(const Eigen::Vector3d& m) const;

private:

	double scalar_;

	Eigen::Vector3d vector_;

};

#endif // MULTIBODYSIM_LIB_QUATERNIONS_QUATERNION_H_