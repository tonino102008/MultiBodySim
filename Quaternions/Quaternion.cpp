#include "Quaternion.h"

Quaternion::Quaternion() :
	scalar(0), vector(std::vector<double>{0, 0, 0}) 
{};

Quaternion::Quaternion(const double s, const std::vector<double> v) :
	scalar(s), vector(v)
{};

double Quaternion::getScalar() const {
	return this->scalar;
};

std::vector<double> Quaternion::getVector() const {
	return this->vector;
};

Quaternion Quaternion::operator+(const Quaternion& q) const {
	Quaternion out;
	out.scalar = this->scalar + q.scalar;
	for (int i = 0; i < this->vector.size(); i++)
		out.vector[i] = this->vector[i] + q.vector[i];
	return out;
};

Quaternion Quaternion::operator*(const Quaternion& q) const {
	Quaternion out;
	out.scalar = this->scalar * q.scalar;
	out.vector[0] = this->vector[1] * q.vector[2] - this->vector[2] * q.vector[1];
	out.vector[1] = this->vector[2] * q.vector[0] - this->vector[0] * q.vector[2];
	out.vector[2] = this->vector[0] * q.vector[1] - this->vector[1] * q.vector[0];
	for (int i = 0; i < this->vector.size(); i++) {
		out.scalar -= this->vector[i] * q.vector[i];
		out.vector[i] = this->scalar * q.vector[i] + q.scalar * this->vector[i];
	}
	return out;
};

Quaternion Quaternion::conj() const {
	Quaternion out;
	out.scalar = this->scalar;
	for (int i = 0; i < this->vector.size(); i++) {
		out.vector[i] = -this->vector[i];
	}
	return out;
};

double Quaternion::norm() const {
	double out = this->scalar * this->scalar;
	for (int i = 0; i < this->vector.size(); i++) {
		out += this->vector[i] * this->vector[i];
	}
	out = sqrt(out);
	return out;
};
