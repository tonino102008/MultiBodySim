#include "MultiBody/External/Damper/Damper.h"

Damper::Damper(const int dof1, const double r) :
	External(dof1), r_(r)
{};

Damper::Damper(const int dof1, const int dof2, const double r) :
	External(dof1, dof2), r_(r)
{};

void Damper::updateExternal(const Eigen::VectorXd& dof, Eigen::VectorXd& f) {
	this->ext_ = this->r_ * (dof.coeff(this->dof2_) - dof.coeff(this->dof1_));
	f.coeffRef(this->dof1_) += this->ext_;
	f.coeffRef(this->dof2_) -= this->ext_;
};