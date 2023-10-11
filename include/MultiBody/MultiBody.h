/*****************************************************************//**
 * \file   MultiBody.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_MULTIBODY_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_MULTIBODY_H_

#include "RigidBody/RigidBody.h"
#include "Constraints/Constraint.h"
#include "External/External.h"
#include "Integrator/Integrator.h"
#include "TimeSim.h"

#include <Eigen/Dense>
#include <vector>
#include <ostream>

class MultiBody {

public:

	/**
	 * @brief 
	 * @param timeStart 
	 * @param timeEnd 
	 * @param dt 
	 * @param timeActual 
	 * @param nBody 
	 * @param nConstr 
	 * @param nExt 
	*/
	MultiBody(const double timeStart, const double timeEnd,
		const double dt, const double timeActual,
		const int nBody, const int nConstr, const int nExt);

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::MatrixXd getdofTimeHistory() const;

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::MatrixXd getMass() const;

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::VectorXd getF() const;

	/**
	 * @brief 
	 * @param time 
	*/
	void setTime(const TimeSim& time);

	/**
	 * @brief 
	 * @param body 
	 * @param i 
	*/
	void setBody(const RigidBody& body, const int i);

	/**
	 * @brief 
	 * @tparam T 
	 * @param constr 
	 * @param i 
	*/
	template <typename T> void setConstr(const T& constr, const int i) {
		this->constraint_[i] = std::make_unique<T>(constr);
	};

	/**
	 * @brief 
	 * @tparam T 
	 * @param ext 
	 * @param i 
	*/
	template <typename T> void setExt(const T& ext, const int i) {
		this->external_[i] = std::make_unique<T>(ext);
	};

	/**
	 * @brief 
	 * @tparam T 
	 * @param integr 
	*/
	template <typename T> void setIntegr(const T& integr) {
		this->integrator_ = std::make_unique<T>(integr);
	};

	/**
	 * @brief 
	*/
	void solve();

	/**
	 * @brief 
	*/
	void printToFile() const;

	/**
	 * @brief 
	*/
	friend std::ostream& operator<<(std::ostream& out, const MultiBody& I);

private:

	Eigen::MatrixXd dofTimeHistory_;

	Eigen::MatrixXd M_;

	Eigen::VectorXd f_;

	std::unique_ptr<TimeSim> time_;

	std::vector<std::unique_ptr<RigidBody>> body_;

	std::vector<std::unique_ptr<Constraint>> constraint_;

	std::vector<std::unique_ptr<External>> external_;

	std::unique_ptr<Integrator> integrator_;

	const int nBody_;

	const int nConstr_;

	const int nExt_ ;
	
};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_MULTIBODY_H_
