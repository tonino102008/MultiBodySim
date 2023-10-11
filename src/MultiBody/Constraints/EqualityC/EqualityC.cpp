#include "MultiBody/Constraints/EqualityC/EqualityC.h"

EqualityC::EqualityC(const int body1, const int body2,
	const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& axis) :
	body1_(body1), body2_(body2), G_(0.0), 
	dGddof_(Eigen::VectorXd(2)), b_(0.0),
	pos1_(pos1), pos2_(pos2), axis_(axis),
	idx_()
{
	axis.maxCoeff(&this->idx_);
};

Eigen::VectorXi EqualityC::getBodyIndex() const {
	return Eigen::VectorXi(this->body1_, this->body2_);
};

double EqualityC::getG() const {
	return this->G_;
};

double EqualityC::getB() const {
	return this->b_;
};

Eigen::VectorXd EqualityC::getDGDDof() const {
	return this->dGddof_;
};

void EqualityC::updateConstraint(const std::vector<std::unique_ptr<RigidBody>>& body,
	Eigen::MatrixXd& M, Eigen::VectorXd& f, const int j) {

	// TBN: The position of the second body is moved together with first body, and viceversa
	// TBN: For a vehicle, the wheels should rotate considering only yaw, NOT pitch and roll

	Eigen::Vector3d pos1G = body[this->body1_]->getDof().segment<3>(7) + body[this->body1_]->getQuaternion().rotateVecG(this->pos1_);

	Eigen::Vector3d pos2G = body[this->body2_]->getDof().segment<3>(7) + body[this->body2_]->getQuaternion().rotateVecG(this->pos2_);

	this->G_ = pos1G.dot(this->axis_) - pos2G.dot(this->axis_); // TBD: things change with posG = pos0G + (posL)G
	this->dGddof_.coeffRef(0) = 1.0;
	this->dGddof_.coeffRef(1) = -1.0;
	Eigen::VectorXd dGddofdt = Eigen::VectorXd::Zero(2);
	this->b_ = dGddofdt.dot((Eigen::VectorXd(2) << body[this->body1_]->getDof().segment<3>(0).dot(this->axis_),
		body[this->body2_]->getDof().segment<3>(0).dot(this->axis_)).finished());

	const int k = body.size() * kDof + j;
	int p1 = this->body1_ * kDof + 7 + this->idx_;
	int p2 = this->body2_ * kDof + 7 + this->idx_;

	M.coeffRef(k, p1) = this->dGddof_.coeff(0);
	M.coeffRef(p1, k) = this->dGddof_.coeff(0);
	M.coeffRef(k, p2) = this->dGddof_.coeff(1);
	M.coeffRef(p2, k) = this->dGddof_.coeff(1);
	M.coeffRef(k, p1 - 7) = 2.0 * kAlpha * this->dGddof_.coeff(0);
	M.coeffRef(p1 - 7, k) = 2.0 * kAlpha * this->dGddof_.coeff(0);
	M.coeffRef(k, p2 - 7) = 2.0 * kAlpha * this->dGddof_.coeff(1);
	M.coeffRef(p2 - 7, k) = 2.0 * kAlpha * this->dGddof_.coeff(1);
	f.coeffRef(k) = this->b_ - kBeta * kBeta * this->G_;
};