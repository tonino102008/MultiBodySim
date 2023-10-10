/*****************************************************************//**
 * \file   EqualityC.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_EQUALITYC_EQUALITYC_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_EQUALITYC_EQUALITYC_H_

#include "MultiBody/Constraints/Constraint.h"
#include "MultiBody/RigidBody/RigidBodyConst.h"

class EqualityC : public Constraint {

public:

	EqualityC(const int dof1);

	EqualityC(const int dof1, const int dof2);

	void updateConstraint(const Eigen::VectorXd& dof,
		Eigen::MatrixXd& M, Eigen::VectorXd& f, const int k);

private:

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_EQUALITYC_EQUALITYC_H_