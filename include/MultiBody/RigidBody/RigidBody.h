#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_RIGIDBODY_RIGIDBODY_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_RIGIDBODY_RIGIDBODY_H_

#include "Quaternion.h"
#include "RigidBodyConst.h"

class RigidBody {

public:

	RigidBody(const Eigen::Matrix3d& m, const Eigen::Matrix3d& J,
		const Eigen::Vector3d& xG0, const Quaternion& q0,
		const Eigen::Vector3d& xGp0, const Quaternion& qp0);

	void updateXG(const Eigen::Vector3d xG);

	void updateXGp(const Eigen::Vector3d xGp);

	void updateQ(const Quaternion q);

	void updateQp(const Quaternion qp);

	void updateLambda(const double lambda);

	void updateDof();

	void updateMass();

	void updateF();

	void updateBody(const Eigen::VectorXd& dof, 
		Eigen::MatrixXd& M, Eigen::VectorXd& f, const int k);

	Eigen::MatrixXd getG() const;

	Eigen::MatrixXd getGp() const;

	Eigen::MatrixXd getE() const;

	Eigen::MatrixXd getEp() const;

	Eigen::VectorXd getDof() const;

	Eigen::MatrixXd getMass() const;

	Eigen::VectorXd getF() const;

	Eigen::VectorXd getWGlobal() const;

	Eigen::VectorXd getWLocal() const;

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