#include "RigidBody/Constraints/EqualityC/EqualityC.h"

EqualityC::EqualityC(const int dof1) :
	Constraint(dof1)
{};

EqualityC::EqualityC(const int dof1, const int dof2) :
	Constraint(dof1, dof2)
{};

void EqualityC::updateConstraint(const MatrixN& dof) {
	this->G_ = dof[this->dof1_][0] - dof[this->dof2_][0];
	int dof1p = this->dof1_ - 7;
	int dof2p = this->dof2_ - 7;
	this->dGddof_[0][0] = 1.0;
	this->dGddof_[1][0] = -1.0;
	MatrixN dGddofdt(2, 1, 0.0);
	this->b_ = dGddofdt.dot(MatrixN(2, 1, { {dof[dof1p][0]}, {dof[dof2p][0]} }));
};