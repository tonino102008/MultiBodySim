#include "RigidBody/VectorN/VectorN.h"

VectorN::VectorN(const int size) :
	kSize_(size), vector_(kSize_)
{};

int VectorN::getSize() const {
	return this->kSize_;
}

double& VectorN::operator[](const int i) {
	if (i > this->getSize()) {
		double out = 0;
		return out;
	}
	return this->vector_[i];
};

double VectorN::operator[](const int i) const {
	if (i > this->getSize()) {
		double out = 0;
		return out;
	}
	return this->vector_[i];
};

VectorN& VectorN::operator=(const VectorN& v) {
	VectorN out(v.getSize());
	out.vector_ = v.vector_;
	return out;
};

VectorN VectorN::operator+(const VectorN& v) const {
	if (this->getSize() != v.getSize()) {
		VectorN out(0);
		return out;
	}
	VectorN out(this->getSize());
	for (int i = 0; i < this->getSize(); i++)
		out[i] = this->vector_[i] + v[i];
	return out;
};

VectorN VectorN::operator-(const VectorN& v) const {
	if (this->getSize() != v.getSize()) {
		VectorN out(0);
		return out;
	}
	VectorN out(this->getSize());
	for (int i = 0; i < this->getSize(); i++)
		out[i] = this->vector_[i] - v[i];
	return out;
};

VectorN VectorN::operator*(const VectorN& v) const {
	if (this->getSize() != v.getSize()) {
		VectorN out(0);
		return out;
	}
	VectorN out(this->getSize());
	for (int i = 0; i < this->getSize(); i++)
		out[i] = this->vector_[i] * v[i];
	return out;
};

VectorN VectorN::operator*(const double d) const {
	VectorN out(this->getSize());
	for (int i = 0; i < this->getSize(); i++)
		out[i] = this->vector_[i] * d;
	return out;
};

std::ostream& operator<<(std::ostream& out, const VectorN& v) {
	for (int i = 0; i < v.getSize(); i++)
		out << v[i] << " ";
	return out;
};

double VectorN::dot(const VectorN& v) const {
	if (this->getSize() != v.getSize()) {
		double out = 0;
		return out;
	}
	double out = 0;
	for (int i = 0; i < this->getSize(); i++)
		out += this->vector_[i] * v[i];
	return out;
};

VectorN VectorN::cross(const VectorN& v) const {
	if ((this->getSize() != 3) || (v.getSize() != 3)) {
		VectorN out(0);
		return out;
	}
	VectorN out(this->getSize());
	out[0] = this->vector_[1] * v[2] - this->vector_[2] * v[1];
	out[1] = this->vector_[2] * v[0] - this->vector_[0] * v[2];
	out[2] = this->vector_[0] * v[1] - this->vector_[1] * v[0];
	return out;
};

double VectorN::norm() const {
	double out = 0;
	for (int i = 0; i < this->getSize(); i++)
		out += this->vector_[i] * this->vector_[i];
	return out;
};