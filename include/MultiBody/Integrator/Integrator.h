/*****************************************************************//**
 * \file   Integrator.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_INTEGRATOR_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_INTEGRATOR_H_

#include "MultiBody/RigidBody/RigidBody.h"
#include "MultiBody/Constraints/Constraint.h"
#include "MultiBody/External/External.h"
#include "TimeSim.h"

#include <memory>
#include <vector>

class Integrator {

public:

	/**
	 * @brief 
	 * @param dofTot 
	 * @param M 
	 * @param f 
	 * @param body 
	 * @param constraint 
	 * @param external 
	 * @param time 
	*/
	virtual void solve(Eigen::MatrixXd& dofTot,
		Eigen::MatrixXd& M, Eigen::VectorXd& f,
		std::vector<std::unique_ptr<RigidBody>>& body,
		std::vector<std::unique_ptr<Constraint>>& constraint,
		std::vector<std::unique_ptr<External>>& external,
		std::unique_ptr<TimeSim>& time) = 0;

	/**
	 * @brief 
	 * @param body 
	 * @param time 
	*/
	virtual void print(std::vector<std::unique_ptr<RigidBody>>& body,
		std::unique_ptr<TimeSim>& time) const = 0;

private:

	/**
	 * @brief 
	 * @param dofTot 
	 * @param M 
	 * @param f 
	 * @param body 
	 * @param constraint 
	 * @param external 
	*/
	virtual void solve0(Eigen::MatrixXd& dofTot,
		Eigen::MatrixXd& M, Eigen::VectorXd& f,
		std::vector<std::unique_ptr<RigidBody>>& body,
		std::vector<std::unique_ptr<Constraint>>& constraint,
		std::vector<std::unique_ptr<External>>& external) = 0;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_INTEGRATOR_H_