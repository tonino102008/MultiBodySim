/*****************************************************************//**
 * \file   RigidBody.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_RIGIDBODY_RIGIDBODY_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_RIGIDBODY_RIGIDBODY_H_

#include "Quaternion.h"
#include "RigidBodyConst.h"

class RigidBody {

public:

	/**
	 * @brief 
	 * @param m 
	 * @param J 
	 * @param xG0 
	 * @param q0 
	 * @param xGp0 
	 * @param qp0 
	*/
	RigidBody(const Eigen::Matrix3d& m, const Eigen::Matrix3d& J,
		const Eigen::Vector3d& xG0, const Quaternion& q0,
		const Eigen::Vector3d& xGp0, const Quaternion& qp0);

	/**
	 * @brief 
	 * @param xG 
	*/
	void updateXG(const Eigen::Vector3d xG);

	/**
	 * @brief 
	 * @param xGp 
	*/
	void updateXGp(const Eigen::Vector3d xGp);

	/**
	 * @brief 
	 * @param q 
	*/
	void updateQ(const Quaternion q);

	/**
	 * @brief 
	 * @param qp 
	*/
	void updateQp(const Quaternion qp);

	/**
	 * @brief 
	 * @param lambda 
	*/
	void updateLambda(const double lambda);

	/**
	 * @brief 
	*/
	void updateDof();

	/**
	 * @brief 
	*/
	void updateMass();

	/**
	 * @brief 
	*/
	void updateF();

	/**
	 * @brief 
	 * @param dof 
	 * @param M 
	 * @param f 
	 * @param k 
	*/
	void updateBody(const Eigen::VectorXd& dof, 
		Eigen::MatrixXd& M, Eigen::VectorXd& f, const int k);

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::MatrixXd getG() const;

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::MatrixXd getGp() const;

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::MatrixXd getE() const;

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::MatrixXd getEp() const;

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::VectorXd getDof() const;

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
	 * @return 
	*/
	Eigen::Vector3d getWGlobal() const;

	/**
	 * @brief 
	 * @return 
	*/
	Eigen::Vector3d getWLocal() const;

	/**
	 * @brief 
	 * @return 
	*/
	Quaternion getQuaternion() const;

private:

	Eigen::Vector3d xG_; // State translation vector [xG,yG,zG]

	Eigen::Vector3d xGp_; // State translation vector [xpG,ypG,zpG]

	Quaternion q_; // State rotation quaternion [qs,qx,qy,qz]

	Quaternion qp_; // State rotation quaternion [qps,qpx,qpy,qpz]

	double lambda_;

	Eigen::VectorXd dof_; // Full State [xpG,ypG,zpG,qps,qpx,qpy,qpz,xG,yG,zG,qs,qx,qy,qz,lambda]

	Eigen::VectorXd dofp_; // Full State [xppG,yppG,zppG,qpps,qppx,qppy,qppz,xpG,ypG,zpG,qps,qpx,qpy,qpz,lambdap]

	Eigen::Matrix3d m_;

	Eigen::Matrix3d J_;

	Eigen::MatrixXd M_;

	Eigen::VectorXd f_;

	Eigen::Vector3d fG_;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_RIGIDBODY_RIGIDBODY_H_