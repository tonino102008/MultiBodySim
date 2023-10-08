#include "MultiBody/Constraints/EqualityC0/EqualityC0.h"

EqualityC0::EqualityC0(const int dof1) :
	Constraint(dof1)
{};

void EqualityC0::updateConstraint(const Eigen::VectorXd& dof, 
	Eigen::MatrixXd& M, Eigen::VectorXd& f, const int k) {
	this->G_ = dof.coeff(this->dof1_);
	int dof1p = this->dof1_ - 7;
	this->dGddof_.coeffRef(0) = 1.0;
	Eigen::VectorXd dGddofdt = Eigen::VectorXd::Zero(1);
	this->b_ = dGddofdt.dot((Eigen::VectorXd(1) << dof[dof1p]).finished());

	M.coeffRef(k, this->dof1_) = this->dGddof_.coeff(0);
	M.coeffRef(this->dof1_, k) = this->dGddof_.coeff(0);
	M.coeffRef(k, dof1p) = 2.0 * kAlpha * this->dGddof_.coeff(0);
	M.coeffRef(dof1p, k) = 2.0 * kAlpha * this->dGddof_.coeff(0);
	f.coeffRef(k) = this->b_ - kBeta * kBeta * this->G_;
};