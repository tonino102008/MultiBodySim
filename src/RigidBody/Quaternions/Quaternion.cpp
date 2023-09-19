#include "RigidBody/Quaternions/Quaternion.h"

Quaternion::Quaternion() :
	scalar_(0), vector_(MatrixN(3,kSingleColumn))
{};

Quaternion::Quaternion(const double s, const MatrixN& m) :
	scalar_(s), vector_(m)
{};

MatrixN Quaternion::getQuaternion() const {
	MatrixN out(4, 1);
	out.fill(0, 0, MatrixN(1, 1, this->getScalar()));
	out.fill(1, 0, this->getVector());
	return out;
};

double Quaternion::getScalar() const {
	return this->scalar_;
};

MatrixN Quaternion::getVector() const {
	return this->vector_;
};

double Quaternion::qx() const {
	return this->vector_[0][0];
};

double Quaternion::qy() const {
	return this->vector_[1][0];
};

double Quaternion::qz() const {
	return this->vector_[2][0];
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
	return out << q.scalar_ << "\n" << q.vector_;
};

Quaternion Quaternion::conj() const {
	return Quaternion(this->scalar_, this->vector_ * (-1.0));
};

double Quaternion::norm() const {
	double out = this->scalar_ * this->scalar_ + this->vector_.norm();
	out = sqrt(out);
	return out;
};
