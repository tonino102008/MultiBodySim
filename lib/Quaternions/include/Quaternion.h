/*****************************************************************//**
 * \file   Quaternion.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_LIB_QUATERNIONS_QUATERNION_H_
#define MULTIBODYSIM_LIB_QUATERNIONS_QUATERNION_H_

#include "Eigen/Dense"

#include <cmath>
#include <ostream>

class Quaternion {

public:

	/**
	 * @brief 
	*/
	Quaternion();

	/**
	 * @brief 
	 * @param s 
	 * @param m 
	*/
	Quaternion(const double s, const Eigen::Vector3d& m);

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::VectorXd getQuaternion() const;

	/**
	 * @brief 
	 * @return 
	*/
	double getScalar() const;

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::Vector3d getVector() const;

	/**
	 * @brief 
	 * @return 
	*/
	double qx() const;

	/**
	 * @brief 
	 * @return 
	*/
	double qy() const;

	/**
	 * @brief 
	 * @return 
	*/
	double qz() const;

	/**
	 * @brief 
	 * @param q 
	 * @return 
	*/
	Quaternion operator+(const Quaternion& q) const;

	/**
	 * @brief 
	 * @param q 
	 * @return 
	*/
	Quaternion operator*(const Quaternion& q) const;

	/**
	 * @brief 
	 * @param out 
	 * @param q 
	 * @return 
	*/
	friend std::ostream& operator<<(std::ostream& out, const Quaternion& q);

	/**
	 * @brief 
	 * @return 
	*/
	Quaternion conj() const;

	/**
	 * @brief 
	 * @return 
	*/
	double norm() const;

	/**
	 * @brief 
	 * @param m 
	 * @return 
	*/
	Eigen::Vector3d rotateVecG(const Eigen::Vector3d& m) const;

	/**
	 * @brief 
	 * @param m 
	 * @return 
	*/
	Eigen::Vector3d rotateVecL(const Eigen::Vector3d& m) const;

private:

	double scalar_;

	Eigen::Vector3d vector_;

};

#endif // MULTIBODYSIM_LIB_QUATERNIONS_QUATERNION_H_