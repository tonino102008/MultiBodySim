#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODY_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODY_H_

#include "Quaternions/Quaternion.h"
#include "RigidBodyConst.h"

class RigidBody {

public:

	RigidBody(const MatrixN& m, const MatrixN& J,
		const MatrixN& xG0, const Quaternion& q0,
		const MatrixN& xGp0, const Quaternion& qp0,
		const MatrixN& fExt, const MatrixN& mExt);

	void updateXG(const MatrixN xG);

	void updateXGp(const MatrixN xGp);

	void updateQ(const Quaternion q);

	void updateQp(const Quaternion qp);

	void updateDof();

	void updateMass();

	void updateF();

	MatrixN getG() const;

	MatrixN getGp() const;

	MatrixN getE() const;

	MatrixN getEp() const;

	MatrixN getDof() const;

	MatrixN getMass() const;

	MatrixN getF() const;

	MatrixN getWGlobal() const;

	MatrixN getWLocal() const;

private:

	MatrixN xG_; // State translation vector [xG,yG,zG]

	MatrixN xGp_; // State translation vector [xpG,ypG,zpG]

	Quaternion q_; // State rotation quaternion [qs,qx,qy,qz]

	Quaternion qp_; // State rotation quaternion [qps,qpx,qpy,qpz]

	MatrixN dof_; // Full State [xpG,ypG,zpG,qps,qpx,qpy,qpz,xG,yG,zG,qs,qx,qy,qz,lambda]

	MatrixN dofp_; // Full State [xppG,yppG,zppG,qpps,qppx,qppy,qppz,xpG,ypG,zpG,qps,qpx,qpy,qpz,lambdap]

	MatrixN m_;

	MatrixN J_;

	MatrixN M_;

	MatrixN f_;

	MatrixN fExt_;

	MatrixN mExt_;

};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODY_H_