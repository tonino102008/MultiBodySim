#include "RigidBody/Quaternions/Quaternion.h"

Quaternion::Quaternion() :
	scalar_(0), vector_(VectorN(3))
{};

Quaternion::Quaternion(const double s, const VectorN v) :
	scalar_(s), vector_(v)
{};

double Quaternion::getScalar() const {
	return this->scalar_;
};

VectorN Quaternion::getVector() const {
	return this->vector_;
};

Quaternion Quaternion::operator+(const Quaternion& q) const {
	Quaternion out;
	out.scalar_ = this->scalar_ + q.scalar_;
	out.vector_ = this->vector_ + q.vector_;
	return out;
};

Quaternion Quaternion::operator*(const Quaternion& q) const {
	Quaternion out;
	out.scalar_ = this->scalar_ * q.scalar_ + this->vector_.dot(q.vector_);
	out.vector_ = this->vector_.cross(q.vector_) + q.vector_ * this->scalar_ + this->vector_ * q.scalar_;
	return out;
};

std::ostream& operator<<(std::ostream& out, const Quaternion& q) {
	return out << q.scalar_ << " " << q.vector_[0] << " " << q.vector_[1] << " " << q.vector_[2];
};

Quaternion Quaternion::conj() const {
	Quaternion out;
	out.scalar_ = this->scalar_;
	out.vector_ = this->vector_ * (- 1.0);
	return out;
};

double Quaternion::norm() const {
	double out = this->scalar_ * this->scalar_ + this->vector_.norm();
	out = sqrt(out);
	return out;
};
