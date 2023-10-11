/*****************************************************************//**
 * \file   Spring.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_SPRING_SPRING_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_SPRING_SPRING_H_

#include "MultiBody/External/External.h"

#include <Eigen/Dense>

class Spring : public External {

public:

	/**
	 * @brief 
	 * @param body1 
	 * @param body2 
	 * @param k 
	 * @param x0 
	 * @param pos1 
	 * @param pos2 
	 * @param axis 
	*/
	Spring(const int body1, const int body2, const double k, const double x0,
		const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& axis);

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::VectorXi getBodyIndex() const;

	/**
	 * @brief 
	 * @return 
	*/
	double getExt() const;

	/**
	 * @brief 
	 * @param body 
	 * @param f 
	*/
	void updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f);

private:

	const int body1_;

	const int body2_;

	double ext_;

	const double k_;

	const double x0_;

	const Eigen::Vector3d pos1_;

	const Eigen::Vector3d pos2_;

	const Eigen::Vector3d axis_;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_SPRING_SPRING_H_