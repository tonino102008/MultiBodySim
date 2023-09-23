#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_CONSTRAINTS_CONSTRAINT_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_CONSTRAINTS_CONSTRAINT_H_

#include "RigidBody/MatrixN/MatrixN.h"

class Constraint {

public:

	Constraint(const int dof1);

	Constraint(const int dof1, const int dof2);

	int getDof1() const;

	int getDof2() const;

	double getG() const;

	MatrixN getDGDDof() const;

	double getB() const;

	virtual void updateConstraint(const MatrixN& dof) = 0;

protected:

	const int dof1_; // TODO: has to become a vector when rotational constraints will be added

	const int dof2_; // TODO: has to become a vector when rotational constraints will be added

	double G_;

	MatrixN dGddof_;

	double b_; // The variable b_ represents: dGddofdt_ * dofp_

};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_CONSTRAINTS_CONSTRAINT_H_