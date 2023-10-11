/*****************************************************************//**
 * \file   External.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_

#include "MultiBody/RigidBody/RigidBody.h"

#include <Eigen/Dense>

class External {

public:

	/**
	 * @brief 
	*/
	External();

	/**
	 * @brief 
	 * @return 
	*/
	virtual Eigen::VectorXi getBodyIndex() const = 0;

	/**
	 * @brief 
	 * @return 
	*/
	virtual double getExt() const = 0;

	/**
	 * @brief 
	 * @param body 
	 * @param f 
	*/
	virtual void updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f) = 0;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_