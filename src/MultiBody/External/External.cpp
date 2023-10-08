#include "MultiBody/External/External.h"

External::External(const int body1) :
	body1_(body1), body2_(-1.0), ext_(0.0)
{};

External::External(const int body1, const int body2) :
	body1_(body1), body2_(body2), ext_(0.0)
{};

int External::getDof1() const {
	return this->body1_;
};

int External::getDof2() const {
	return this->body2_;
};

double External::getExt() const {
	return this->ext_;
};