#include "RigidBody/Quaternions/Quaternion.h"

Quaternion::Quaternion() :
	scalar_(0), vector_(Eigen::Vector3d::Zero())
{};

Quaternion::Quaternion(const double s, const Eigen::Vector3d& m) :
	scalar_(s), vector_(m)
{};

Eigen::VectorXd Quaternion::getQuaternion() const {
	Eigen::VectorXd out(4);
	out << this->scalar_, this->vector_;
	return out;
};

double Quaternion::getScalar() const {
	return this->scalar_;
};

Eigen::Vector3d Quaternion::getVector() const {
	return this->vector_;
};

double Quaternion::qx() const {
	return this->vector_.coeff(0);
};

double Quaternion::qy() const {
	return this->vector_.coeff(1);
};

double Quaternion::qz() const {
	return this->vector_.coeff(2);
};

Quaternion Quaternion::operator+(const Quaternion& q) const {
	return Quaternion(this->scalar_ + q.scalar_,
		this->vector_ + q.vector_);
};

Quaternion Quaternion::operator*(const Quaternion& q) const {
	return Quaternion(this->scalar_ * q.scalar_ - this->vector_.dot(q.vector_),
		this->vector_.cross(q.vector_) + q.vector_ * this->scalar_ + this->vector_ * q.scalar_);
};

std::ostream& operator<<(std::ostream& out, const Quaternion& q) {
	return out << q.getQuaternion();
};

Quaternion Quaternion::conj() const {
	return Quaternion(this->scalar_, -1.0 * this->vector_);
};

double Quaternion::norm() const {
	return sqrt(this->scalar_ * this->scalar_ + this->vector_.dot(this->vector_));
};
