#include "MultiBody/Constraints/Constraint.h"

Constraint::Constraint(const int dof1) :
	dof1_(dof1), dof2_(-1.0), G_(0.0), dGddof_(Eigen::VectorXd(1)), b_(0.0)
{};

Constraint::Constraint(const int dof1, const int dof2) :
	dof1_(dof1), dof2_(dof2), G_(0.0), dGddof_(Eigen::VectorXd(2)), b_(0.0)
{};

int Constraint::getDof1() const {
	return this->dof1_;
};

int Constraint::getDof2() const {
	return this->dof2_;
};

double Constraint::getG() const {
	return this->G_;
};

Eigen::VectorXd Constraint::getDGDDof() const {
	return this->dGddof_;
};

double Constraint::getB() const {
	return this->b_;
};