#include "Quaternions/Quaternion.h"

Quaternion::Quaternion() :
	scalar_(0), vector_(std::vector<double>{0, 0, 0}) 
{};

Quaternion::Quaternion(const double s, const std::vector<double> v) :
	scalar_(s), vector_(v)
{};

double Quaternion::getScalar() const {
	return this->scalar_;
};

std::vector<double> Quaternion::getVector() const {
	return this->vector_;
};

Quaternion Quaternion::operator+(const Quaternion& q) const {
	Quaternion out;
	out.scalar_ = this->scalar_ + q.scalar_;
	for (int i = 0; i < this->vector_.size(); i++)
		out.vector_[i] = this->vector_[i] + q.vector_[i];
	return out;
};

Quaternion Quaternion::operator*(const Quaternion& q) const {
	Quaternion out;
	out.scalar_ = this->scalar_ * q.scalar_;
	out.vector_[0] = this->vector_[1] * q.vector_[2] - this->vector_[2] * q.vector_[1];
	out.vector_[1] = this->vector_[2] * q.vector_[0] - this->vector_[0] * q.vector_[2];
	out.vector_[2] = this->vector_[0] * q.vector_[1] - this->vector_[1] * q.vector_[0];
	for (int i = 0; i < this->vector_.size(); i++) {
		out.scalar_ -= this->vector_[i] * q.vector_[i];
		out.vector_[i] = this->scalar_ * q.vector_[i] + q.scalar_ * this->vector_[i];
	}
	return out;
};

std::ostream& operator<<(std::ostream& out, const Quaternion& q) {
	return out << q.scalar_ << " " << q.vector_[0] << " " << q.vector_[1] << " " << q.vector_[2];
};

Quaternion Quaternion::conj() const {
	Quaternion out;
	out.scalar_ = this->scalar_;
	for (int i = 0; i < this->vector_.size(); i++) {
		out.vector_[i] = -this->vector_[i];
	}
	return out;
};

double Quaternion::norm() const {
	double out = this->scalar_ * this->scalar_;
	for (int i = 0; i < this->vector_.size(); i++) {
		out += this->vector_[i] * this->vector_[i];
	}
	out = sqrt(out);
	return out;
};
