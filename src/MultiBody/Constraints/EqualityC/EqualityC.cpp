#include "MultiBody/Constraints/EqualityC/EqualityC.h"

EqualityC::EqualityC(const int dof1) :
	Constraint(dof1)
{};

EqualityC::EqualityC(const int dof1, const int dof2) :
	Constraint(dof1, dof2)
{};

void EqualityC::updateConstraint(const Eigen::VectorXd& dof) {
	this->G_ = dof[this->dof1_] - dof[this->dof2_];
	int dof1p = this->dof1_ - 7;
	int dof2p = this->dof2_ - 7;
	this->dGddof_[0] = 1.0;
	this->dGddof_[1] = -1.0;
	Eigen::VectorXd dGddofdt = Eigen::VectorXd::Zero(2);
	this->b_ = dGddofdt.dot((Eigen::VectorXd(2) << dof[dof1p], dof[dof2p]).finished());
};