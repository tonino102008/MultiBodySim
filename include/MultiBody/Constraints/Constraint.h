/*****************************************************************//**
 * \file   Constraint.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_CONSTRAINT_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_CONSTRAINT_H_

#include "MultiBody/RigidBody/RigidBody.h"

#include <Eigen/Dense>

class Constraint {

public:

	/**
	 * @brief 
	*/
	Constraint();
	
	/**
	 * @brief 
	 * @return 
	*/
	virtual Eigen::VectorXi getBodyIndex() const = 0;

	/**
	 * @brief
	 * @return
	*/
	virtual double getG() const = 0;

	/**
	 * @brief
	 * @return
	*/
	virtual double getB() const = 0;

	/**
	 * @brief 
	 * @return 
	*/
	virtual Eigen::VectorXd getDGDDof() const = 0;

	/**
	 * @brief 
	 * @param body 
	 * @param M 
	 * @param f 
	 * @param j 
	*/
	virtual void updateConstraint(const std::vector<std::unique_ptr<RigidBody>>& body,
		Eigen::MatrixXd& M, Eigen::VectorXd& f, const int j) = 0;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_CONSTRAINT_H_