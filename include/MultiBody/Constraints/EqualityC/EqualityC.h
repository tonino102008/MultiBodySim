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

	/**
	 * @brief 
	 * @param body1 
	 * @param body2 
	 * @param pos1 
	 * @param pos2 
	 * @param axis 
	*/
	EqualityC(const int body1, const int body2,
		const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2,
		const Eigen::Vector3d& axis);

	/**
	 * @brief
	 * @return
	*/
	Eigen::VectorXi getBodyIndex() const;

	/**
	 * @brief
	 * @return
	*/
	double getG() const;

	/**
	 * @brief
	 * @return
	*/
	double getB() const;

	/**
	 * @brief
	 * @return
	*/
	Eigen::VectorXd getDGDDof() const;

	/**
	 * @brief 
	 * @param body 
	 * @param M 
	 * @param f 
	 * @param j 
	*/
	void updateConstraint(const std::vector<std::unique_ptr<RigidBody>>& body,
		Eigen::MatrixXd& M, Eigen::VectorXd& f, const int j);

private:

	const int body1_;

	const int body2_;

	double G_;

	Eigen::VectorXd dGddof_;

	double b_; // The variable b_ represents: dGddofdt_ * dofp_

	const Eigen::Vector3d pos1_;

	const Eigen::Vector3d pos2_;

	const Eigen::Vector3d axis_;

	int idx_;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_CONSTRAINTS_EQUALITYC_EQUALITYC_H_