#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_CONSTRAINTS_EQUALITYC_EQUALITYC_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_CONSTRAINTS_EQUALITYC_EQUALITYC_H_

#include "RigidBody/Constraints/Constraints.h"

class EqualityC : public Constraint {

public:

	EqualityC(const int dof1);

	EqualityC(const int dof1, const int dof2);

	void updateConstraint(const Eigen::VectorXd& dof);

private:

};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_CONSTRAINTS_EQUALITYC_EQUALITYC_H_