#include "MultiBody/External/Spring/Spring.h"

Spring::Spring(const int dof1, const double k, const double x0) :
	External(dof1), k_(k), x0_(x0)
{};

Spring::Spring(const int dof1, const int dof2, const double k, const double x0) :
	External(dof1, dof2), k_(k), x0_(x0)
{};

void Spring::updateExt(const Eigen::VectorXd& dof) {
	this->ext_ = this->k_ * (this->x0_ + dof.coeff(this->dof2_) - dof.coeff(this->dof1_));
};