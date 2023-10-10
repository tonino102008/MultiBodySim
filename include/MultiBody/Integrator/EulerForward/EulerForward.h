/*****************************************************************//**
 * \file   EulerForward.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_EULERFORWARD_EULERFORWARD_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_EULERFORWARD_EULERFORWARD_H_

#include "MultiBody/Integrator/Integrator.h"

#include <Eigen/Dense>

class EulerForward : public Integrator {

public:

	/**
	 * @brief 
	*/
	EulerForward();

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
	void solve(Eigen::MatrixXd& dofTot,
		Eigen::MatrixXd& M, Eigen::VectorXd& f, 
		std::vector<std::unique_ptr<RigidBody>>& body,
		std::vector<std::unique_ptr<Constraint>>& constraint,
		std::vector<std::unique_ptr<External>>& external,
		std::unique_ptr<TimeSim>& time);

	/**
	 * @brief 
	 * @param body 
	 * @param time 
	*/
	void print(std::vector<std::unique_ptr<RigidBody>>& body,
		std::unique_ptr<TimeSim>& time) const;

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
	void solve0(Eigen::MatrixXd& dofTot,
		Eigen::MatrixXd& M, Eigen::VectorXd& f,
		std::vector<std::unique_ptr<RigidBody>>& body,
		std::vector<std::unique_ptr<Constraint>>& constraint,
		std::vector<std::unique_ptr<External>>& external);

};

#endif //MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_EULERFORWARD_EULERFORWARD_H_