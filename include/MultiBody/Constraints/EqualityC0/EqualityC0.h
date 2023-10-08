#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_EQUALITYC0_EQUALITYC0_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_EQUALITYC0_EQUALITYC0_H_

#include "MultiBody/Constraints/Constraint.h"
#include "MultiBody/RigidBody/RigidBodyConst.h"

class EqualityC0 : public Constraint {

public:

	EqualityC0(const int dof1);

	void updateConstraint(const Eigen::VectorXd& dof,
		Eigen::MatrixXd& M, Eigen::VectorXd& f, const int k);

private:

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_EQUALITYC0_EQUALITYC0_H_