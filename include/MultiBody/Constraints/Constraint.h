/*****************************************************************//**
 * \file   Constraint.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_CONSTRAINT_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_CONSTRAINT_H_

#include <Eigen/Dense>

class Constraint {

public:

	/**
	 * @brief 
	 * @param dof1 
	*/
	Constraint(const int dof1);

	/**
	 * @brief 
	 * @param dof1 
	 * @param dof2 
	*/
	Constraint(const int dof1, const int dof2);

	/**
	 * @brief 
	 * @return 
	*/
	int getDof1() const;

	/**
	 * @brief 
	 * @return 
	*/
	int getDof2() const;

	/**
	 * @brief 
	 * @return 
	*/
	double getG() const;

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::VectorXd getDGDDof() const;

	/**
	 * @brief 
	 * @return 
	*/
	double getB() const;

	/**
	 * @brief 
	 * @param dof 
	 * @param M 
	 * @param f 
	 * @param k 
	*/
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