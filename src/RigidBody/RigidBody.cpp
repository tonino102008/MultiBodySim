#include "RigidBody/RigidBody.h"

RigidBody::RigidBody(const MatrixN& m, const MatrixN& J,
	const MatrixN& xG0, const Quaternion& q0,
	const MatrixN& xGp0, const Quaternion& qp0) :
	m_(m), J_(J), xG_(xG0), q_(q0), xGp_(xGp0), qp_(qp0),
	dof_(MatrixN(15, 1)), dofp_(MatrixN(15, 1)),
	M_(MatrixN(15, 15)), f_(MatrixN(15, 1))
{
	MatrixN qs(1, 1, this->q_.getScalar());
	MatrixN qsp(1, 1, this->qp_.getScalar());
	this->dof_.fill(0, 0, xGp_);
	this->dof_.fill(3, 0, qsp);
	this->dof_.fill(4, 0, this->qp_.getVector());
	this->dof_.fill(7, 0, xG_);
	this->dof_.fill(10, 0, qs);
	this->dof_.fill(11, 0, this->q_.getVector());

	this->M_.fill(0, 0, m_);
	this->M_.fill(3, 3, J_);
	this->M_.fill(7, 0, MatrixN(7, 1.0));
};

MatrixN RigidBody::getDof() const {
	return this->dof_;
};

MatrixN RigidBody::getMass () const {
	return this->M_;
};