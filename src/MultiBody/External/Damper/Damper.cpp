#include "MultiBody/External/Damper/Damper.h"

Damper::Damper(const int dof1, const double r) :
	External(dof1), r_(r)
{};

Damper::Damper(const int dof1, const int dof2, const double r) :
	External(dof1, dof2), r_(r)
{};

void Damper::updateExt(const Eigen::VectorXd& dof) {
	this->ext_ = this->r_ * (dof.coeff(this->dof2_) - dof.coeff(this->dof1_));
};