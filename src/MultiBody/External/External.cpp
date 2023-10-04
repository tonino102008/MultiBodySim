#include "MultiBody/External/External.h"

External::External(const int dof1) :
	dof1_(dof1), dof2_(-1.0), ext_(0.0)
{};

External::External(const int dof1, const int dof2) :
	dof1_(dof1), dof2_(dof2), ext_(0.0)
{};

int External::getDof1() const {
	return this->dof1_;
};

int External::getDof2() const {
	return this->dof2_;
};

double External::getExt() const {
	return this->ext_;
};