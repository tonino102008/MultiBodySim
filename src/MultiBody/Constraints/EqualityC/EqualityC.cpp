#include "MultiBody/Constraints/EqualityC/EqualityC.h"

EqualityC::EqualityC(const int dof1) :
	Constraint(dof1)
{};

EqualityC::EqualityC(const int dof1, const int dof2) :
	Constraint(dof1, dof2)
{};

void EqualityC::updateConstraint(const Eigen::VectorXd& dof, 
	Eigen::MatrixXd& M, Eigen::VectorXd& f, const int k) {
	this->G_ = dof.coeff(this->dof1_) - dof.coeff(this->dof2_);
	int dof1p = this->dof1_ - 7;
	int dof2p = this->dof2_ - 7;
	this->dGddof_.coeffRef(0) = 1.0;
	this->dGddof_.coeffRef(1) = -1.0;
	Eigen::VectorXd dGddofdt = Eigen::VectorXd::Zero(2);
	this->b_ = dGddofdt.dot((Eigen::VectorXd(2) << dof[dof1p], dof[dof2p]).finished());

	M.coeffRef(k, this->dof1_) = this->dGddof_.coeff(0);
	M.coeffRef(this->dof1_, k) = this->dGddof_.coeff(0);
	M.coeffRef(k, this->dof2_) = this->dGddof_.coeff(1);
	M.coeffRef(this->dof2_, k) = this->dGddof_.coeff(1);
	M.coeffRef(k, dof1p) = 2.0 * kAlpha * this->dGddof_.coeff(0);
	M.coeffRef(dof1p, k) = 2.0 * kAlpha * this->dGddof_.coeff(0);
	M.coeffRef(k, dof2p) = 2.0 * kAlpha * this->dGddof_.coeff(1);
	M.coeffRef(dof2p, k) = 2.0 * kAlpha * this->dGddof_.coeff(1);
	f.coeffRef(k) = this->b_ - kBeta * kBeta * this->G_;
};