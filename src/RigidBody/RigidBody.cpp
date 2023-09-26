#include "RigidBody/RigidBody.h"

RigidBody::RigidBody(const Eigen::Matrix3d& m, const Eigen::Matrix3d& J,
	const Eigen::Vector3d& xG0, const Quaternion& q0,
	const Eigen::Vector3d& xGp0, const Quaternion& qp0,
	const Eigen::Vector3d& fExt, const Eigen::Vector3d& mExt) :
	m_(m), J_(J), xG_(xG0), q_(q0), xGp_(xGp0), qp_(qp0), lambda_(0.0),
	dof_(Eigen::VectorXd::Zero(15)), dofp_(Eigen::VectorXd::Zero(15)),
	M_(Eigen::MatrixXd::Zero(15, 15)), f_(Eigen::VectorXd::Zero(15)),
	fExt_(fExt), mExt_(mExt)
{
	this->updateDof();

	this->M_.block<3, 3>(0, 0) = m_;
	this->updateMass();
	this->M_.block<7, 7>(7, 7) = Eigen::MatrixXd::Identity(7, 7);

	this->updateF();
};

void RigidBody::updateXG(const Eigen::Vector3d xG) {
	this->xG_ = xG;
};

void RigidBody::updateXGp(const Eigen::Vector3d xGp) {
	this->xGp_ = xGp;
};

void RigidBody::updateQ(const Quaternion q) {
	this->q_ = q;
};

void RigidBody::updateQp(const Quaternion qp) {
	this->qp_ = qp;
};

void RigidBody::updateLambda(const double lambda) {
	this->lambda_ = lambda;
};

void RigidBody::updateDof() {
	this->dof_.segment<3>(0) = this->xGp_;
	this->dof_.segment<4>(3) = this->qp_.getQuaternion();
	this->dof_.segment<3>(7) = this->xG_;
	this->dof_.segment<4>(10) = this->q_.getQuaternion();
	this->dof_.coeffRef(14) = this->lambda_;
};

void RigidBody::updateMass() {
	Eigen::MatrixXd G = this->getG();
	double qs = this->q_.getScalar();
	double qx = this->q_.qx();
	double qy = this->q_.qy();
	double qz = this->q_.qz();
	this->M_.block<4, 4>(3, 3) = 4.0 * G.transpose() * this->J_ * G;
	this->M_.coeffRef(3, 14) = qs;
	this->M_.coeffRef(4, 14) = qx;
	this->M_.coeffRef(5, 14) = qy;
	this->M_.coeffRef(6, 14) = qz;
	this->M_.coeffRef(14, 3) = qs;
	this->M_.coeffRef(14, 4) = qx;
	this->M_.coeffRef(14, 5) = qy;
	this->M_.coeffRef(14, 6) = qz;
	this->M_.coeffRef(14, 10) = 2.0 * kAlpha * qs;
	this->M_.coeffRef(14, 11) = 2.0 * kAlpha * qx;
	this->M_.coeffRef(14, 12) = 2.0 * kAlpha * qy;
	this->M_.coeffRef(14, 13) = 2.0 * kAlpha * qz;
};

void RigidBody::updateF() {
	Eigen::MatrixXd G = this->getG();
	Eigen::MatrixXd Gp = this->getGp();
	Eigen::VectorXd qp = this->qp_.getQuaternion();
	this->f_.segment<3>(0) = this->fExt_; // External Forces
	this->f_.segment<4>(3) = -8.0 * Gp.transpose() * this->J_ * G * qp + 2.0 * G.transpose() * this->mExt_; // Other + External Momenta
	this->f_.coeffRef(3) += 4.0 * qp.transpose() * kDGDqs.transpose() * this->J_ * G * qp;
	this->f_.coeffRef(4) += 4.0 * qp.transpose() * kDGDqx.transpose() * this->J_ * G * qp;
	this->f_.coeffRef(5) += 4.0 * qp.transpose() * kDGDqy.transpose() * this->J_ * G * qp;
	this->f_.coeffRef(6) += 4.0 * qp.transpose() * kDGDqz.transpose() * this->J_ * G * qp;
	this->f_.segment<3>(7) = this->xGp_;
	this->f_.segment<4>(10) = qp;
	this->f_.coeffRef(14) = - this->qp_.norm() * this->qp_.norm() + kBeta * kBeta * (1.0 - this->q_.norm());
};

Eigen::MatrixXd RigidBody::getG() const {
	double qs = this->q_.getScalar();
	double qx = this->q_.qx();
	double qy = this->q_.qy();
	double qz = this->q_.qz();
	Eigen::MatrixXd out(3, 4);

	out << -qx, qs, qz, -qy,
		-qy, -qz, qs, qx,
		-qz, qy, -qx, qs;

	return out;
};

Eigen::MatrixXd RigidBody::getGp() const {
	double qps = this->qp_.getScalar();
	double qpx = this->qp_.qx();
	double qpy = this->qp_.qy();
	double qpz = this->qp_.qz();
	Eigen::MatrixXd out(3, 4);

	out << -qpx, qps, qpz, -qpy,
		-qpy, -qpz, qps, qpx,
		-qpz, qpy, -qpx, qps;

	return out;
};

Eigen::MatrixXd RigidBody::getE() const {
	double qs = this->q_.getScalar();
	double qx = this->q_.qx();
	double qy = this->q_.qy();
	double qz = this->q_.qz();
	Eigen::MatrixXd out(3, 4);

	out << -qx, qs, -qz, qy,
		-qy, qz, qs, -qx,
		-qz, -qy, qx, qs;

	return out;
};

Eigen::MatrixXd RigidBody::getEp() const {
	double qps = this->qp_.getScalar();
	double qpx = this->qp_.qx();
	double qpy = this->qp_.qy();
	double qpz = this->qp_.qz();
	Eigen::MatrixXd out(3, 4);

	 out << -qpx, qps, -qpz, qpy,
		 -qpy, qpz, qps, -qpx,
		 -qpz, -qpy, qpx, qps;

	return out;
};

Eigen::VectorXd RigidBody::getDof() const {
	return this->dof_;
};

Eigen::MatrixXd RigidBody::getMass() const {
	return this->M_;
};

Eigen::VectorXd RigidBody::getF() const {
	return this->f_;
};

Eigen::VectorXd RigidBody::getWGlobal() const {
	return (this->qp_ * this->q_.conj()).getQuaternion() * 2.0;
};

Eigen::VectorXd RigidBody::getWLocal() const {
	return (this->q_.conj() * this->qp_).getQuaternion() * 2.0;
};