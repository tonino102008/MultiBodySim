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

void RigidBody::updateXG(const MatrixN xG) {
	this->xG_ = xG;
};

void RigidBody::updateXGp(const MatrixN xGp) {
	this->xGp_ = xGp;
};

void RigidBody::updateQ(const Quaternion q) {
	this->q_ = q;
};

void RigidBody::updateQp(const Quaternion qp) {
	this->qp_ = qp;
};

void RigidBody::updateDof() {
	this->dof_.fill(0, 0, this->xGp_);
	this->dof_.fill(3, 0, this->qp_.getQuaternion());
	this->dof_.fill(7, 0, this->xG_);
	this->dof_.fill(10, 0, this->q_.getQuaternion());
};

void RigidBody::updateMass() {
	MatrixN G = this->getG();
	double qs = this->q_.getScalar();
	double qx = this->q_.qx();
	double qy = this->q_.qy();
	double qz = this->q_.qz();
	this->M_.fill(3, 3, (((G.T()) * this->J_) * G) * 4.0);
	this->M_[3][14] = qs;
	this->M_[4][14] = qx;
	this->M_[5][14] = qy;
	this->M_[6][14] = qz;
	this->M_[14][3] = qs;
	this->M_[14][4] = qx;
	this->M_[14][5] = qy;
	this->M_[14][6] = qz;
	this->M_[14][10] = 2 * kAlpha * qs;
	this->M_[14][11] = 2 * kAlpha * qx;
	this->M_[14][12] = 2 * kAlpha * qy;
	this->M_[14][13] = 2 * kAlpha * qz;
};

void RigidBody::updateF() {
	MatrixN G = this->getG();
	MatrixN Gp = this->getGp();
	MatrixN qp = this->qp_.getQuaternion();
	this->f_.fill(0, 0, MatrixN(3, 1, { {0.0}, {0.0}, {0.0} })); // External Forces
	this->f_.fill(3, 0, ((((Gp.T()) * this->J_) * G) * (-8.0)) * qp + (G.T() * MatrixN(3, 1, { {0.0}, {0.0}, {0.0} })) * 2.0); // Other + External Momenta
	this->f_[3][0] += ((((((qp.T()) * kDGDqs.T()) * this->J_) * G) * qp) * 4.0)[0][0];
	this->f_[4][0] += ((((((qp.T()) * kDGDqx.T()) * this->J_) * G) * qp) * 4.0)[0][0];
	this->f_[5][0] += ((((((qp.T()) * kDGDqy.T()) * this->J_) * G) * qp) * 4.0)[0][0];
	this->f_[6][0] += ((((((qp.T()) * kDGDqz.T()) * this->J_) * G) * qp) * 4.0)[0][0];
	this->f_.fill(7, 0, this->xGp_);
	this->f_.fill(10, 0, qp);
	this->f_[14][0] = - ((this->qp_.norm() * this->qp_.norm()) - (kBeta * kBeta) + ((kBeta * kBeta) * this->q_.norm()));
};

MatrixN RigidBody::getG() const {
	MatrixN out(3, 4);
	double qs = this->q_.getScalar();
	double qx = this->q_.qx();
	double qy = this->q_.qy();
	double qz = this->q_.qz();

	out[0][0] = -qx;
	out[0][1] = qs;
	out[0][2] = qz;
	out[0][3] = -qy;

	out[1][0] = -qy;
	out[1][1] = -qz;
	out[1][2] = qs;
	out[1][3] = qx;

	out[2][0] = -qz;
	out[2][1] = qy;
	out[2][2] = -qx;
	out[2][3] = qs;

	return out;
};

MatrixN RigidBody::getGp() const {
	MatrixN out(3, 4);
	double qps = this->qp_.getScalar();
	double qpx = this->qp_.qx();
	double qpy = this->qp_.qy();
	double qpz = this->qp_.qz();

	out[0][0] = -qpx;
	out[0][1] = qps;
	out[0][2] = qpz;
	out[0][3] = -qpy;

	out[1][0] = -qpy;
	out[1][1] = -qpz;
	out[1][2] = qps;
	out[1][3] = qpx;

	out[2][0] = -qpz;
	out[2][1] = qpy;
	out[2][2] = -qpx;
	out[2][3] = qps;

	return out;
};

MatrixN RigidBody::getE() const {
	MatrixN out(3, 4);
	double qs = this->q_.getScalar();
	double qx = this->q_.qx();
	double qy = this->q_.qy();
	double qz = this->q_.qz();

	out[0][0] = -qx;
	out[0][1] = qs;
	out[0][2] = -qz;
	out[0][3] = qy;

	out[1][0] = -qy;
	out[1][1] = qz;
	out[1][2] = qs;
	out[1][3] = -qx;

	out[2][0] = -qz;
	out[2][1] = -qy;
	out[2][2] = qx;
	out[2][3] = qs;

	return out;
};

MatrixN RigidBody::getEp() const {
	MatrixN out(3, 4);
	double qps = this->qp_.getScalar();
	double qpx = this->qp_.qx();
	double qpy = this->qp_.qy();
	double qpz = this->qp_.qz();

	out[0][0] = -qpx;
	out[0][1] = qps;
	out[0][2] = -qpz;
	out[0][3] = qpy;

	out[1][0] = -qpy;
	out[1][1] = qpz;
	out[1][2] = qps;
	out[1][3] = -qpx;

	out[2][0] = -qpz;
	out[2][1] = -qpy;
	out[2][2] = qpx;
	out[2][3] = qps;

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