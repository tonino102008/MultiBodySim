#include "RigidBody/RigidBody.h"

RigidBody::RigidBody(const MatrixN& m, const MatrixN& J,
	const MatrixN& xG0, const Quaternion& q0,
	const MatrixN& xGp0, const Quaternion& qp0) :
	m_(m), J_(J), xG_(xG0), q_(q0), xGp_(xGp0), qp_(qp0),
	dof_(MatrixN(15, 1)), dofp_(MatrixN(15, 1)),
	M_(MatrixN(15, 15)), f_(MatrixN(15, 1))
{
	this->updateDof();

	this->M_.fill(0, 0, m_);
	this->updateMass();
	this->M_.fill(7, 7, MatrixN(7, 1.0));

	this->updateF();
};

void RigidBody::updateDof() {
	this->dof_.fill(0, 0, this->xGp_);
	this->dof_.fill(3, 0, this->qp_.getQuaternion());
	this->dof_.fill(7, 0, this->xG_);
	this->dof_.fill(10, 0, this->q_.getQuaternion());
};

void RigidBody::updateMass() {
	MatrixN G = this->getG();
	this->M_.fill(3, 3, (((G.T()) * this->J_) * G) * 4.0);
};

void RigidBody::updateF() {
	MatrixN G = this->getG();
	MatrixN Gp = this->getGp();
	this->f_.fill(3, 0, ((((Gp.T()) * this->J_) * G) * (-8.0)) * qp_.getQuaternion());
	this->f_[3][0] += ((((((qp_.getQuaternion().T()) * kDGDqs.T()) * this->J_) * G) * qp_.getQuaternion()) * 4.0)[0][0];
	this->f_[4][0] += ((((((qp_.getQuaternion().T()) * kDGDqx.T()) * this->J_) * G) * qp_.getQuaternion()) * 4.0)[0][0];
	this->f_[5][0] += ((((((qp_.getQuaternion().T()) * kDGDqy.T()) * this->J_) * G) * qp_.getQuaternion()) * 4.0)[0][0];
	this->f_[6][0] += ((((((qp_.getQuaternion().T()) * kDGDqz.T()) * this->J_) * G) * qp_.getQuaternion()) * 4.0)[0][0];
	this->f_.fill(7, 0, this->xGp_);
	this->f_.fill(10, 0, qp_.getQuaternion());
};

MatrixN RigidBody::getG() const {
	MatrixN out(3, 4);

	out[0][0] = -this->q_.qx();
	out[0][1] = this->q_.getScalar();
	out[0][2] = this->q_.qz();
	out[0][3] = -this->q_.qy();

	out[1][0] = -this->q_.qy();
	out[1][1] = -this->q_.qz();
	out[1][2] = this->q_.getScalar();
	out[1][3] = this->q_.qx();

	out[2][0] = -this->q_.qz();
	out[2][1] = this->q_.qy();
	out[2][2] = -this->q_.qx();
	out[2][3] = this->q_.getScalar();

	return out;
};

MatrixN RigidBody::getGp() const {
	MatrixN out(3, 4);

	out[0][0] = -this->qp_.qx();
	out[0][1] = this->qp_.getScalar();
	out[0][2] = this->qp_.qz();
	out[0][3] = -this->qp_.qy();

	out[1][0] = -this->qp_.qy();
	out[1][1] = -this->qp_.qz();
	out[1][2] = this->qp_.getScalar();
	out[1][3] = this->qp_.qx();

	out[2][0] = -this->qp_.qz();
	out[2][1] = this->qp_.qy();
	out[2][2] = -this->qp_.qx();
	out[2][3] = this->qp_.getScalar();

	return out;
};

MatrixN RigidBody::getDof() const {
	return this->dof_;
};

MatrixN RigidBody::getMass() const {
	return this->M_;
};

MatrixN RigidBody::getF() const {
	return this->f_;
};

MatrixN RigidBody::getWGlobal() const {
	return (this->qp_ * (this->q_.conj())).getQuaternion() * 2.0;
};

MatrixN RigidBody::getWLocal() const {
	return ((this->q_.conj()) * this->qp_).getQuaternion() * 2.0;
};