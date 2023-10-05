#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_CONSTRAINT_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_CONSTRAINT_H_

#include <Eigen/Dense>

class Constraint {

public:

	Constraint(const int dof1);

	Constraint(const int dof1, const int dof2);

	int getDof1() const;

	int getDof2() const;

	double getG() const;

	Eigen::VectorXd getDGDDof() const;

	double getB() const;

	virtual void updateConstraint(const Eigen::VectorXd& dof,
		Eigen::MatrixXd& M, Eigen::VectorXd& f, const int k) = 0;

protected:

	const int dof1_; // TODO: has to become a vector when rotational constraints will be added

	const int dof2_; // TODO: has to become a vector when rotational constraints will be added

	double G_;

	Eigen::VectorXd dGddof_;

	double b_; // The variable b_ represents: dGddofdt_ * dofp_

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_CONSTRAINT_H_