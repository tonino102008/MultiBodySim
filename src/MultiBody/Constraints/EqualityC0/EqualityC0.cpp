#include "MultiBody/Constraints/EqualityC0/EqualityC0.h"

EqualityC0::EqualityC0(const int body1, const Eigen::Vector3d& pos1,
	const Eigen::Vector3d& pos2, const Eigen::Vector3d& axis) :
	body1_(body1), G_(0.0),	dGddof_(Eigen::VectorXd(1)),
	b_(0.0), pos1_(pos1), pos2_(pos2), axis_(axis), idx_()
{
	axis.maxCoeff(&this->idx_);
};

Eigen::VectorXi EqualityC0::getBodyIndex() const {
	return Eigen::VectorXi(this->body1_);
};

double EqualityC0::getG() const {
	return this->G_;
};

double EqualityC0::getB() const {
	return this->b_;
};

Eigen::VectorXd EqualityC0::getDGDDof() const {
	return this->dGddof_;
};

void EqualityC0::updateConstraint(const std::vector<std::unique_ptr<RigidBody>>& body,
	Eigen::MatrixXd& M, Eigen::VectorXd& f, const int j) {

	Eigen::Vector3d pos1G = body[this->body1_]->getDof().segment<3>(7) + body[this->body1_]->getQuaternion().rotateVecG(this->pos1_);

	this->G_ = pos1G.dot(this->axis_) - this->pos2_.dot(this->axis_);
	this->dGddof_.coeffRef(0) = 1.0;
	Eigen::VectorXd dGddofdt = Eigen::VectorXd::Zero(1);
	this->b_ = dGddofdt.dot((Eigen::VectorXd(1) << body[this->body1_]->getDof().segment<3>(0).dot(this->axis_)).finished());

	const int k = body.size() * kDof + j;
	int p1 = this->body1_ * kDof + 7 + this->idx_;

	M.coeffRef(k, p1) = this->dGddof_.coeff(0);
	M.coeffRef(p1, k) = this->dGddof_.coeff(0);
	M.coeffRef(k, p1 - 7) = 2.0 * kAlpha * this->dGddof_.coeff(0);
	M.coeffRef(p1 - 7, k) = 2.0 * kAlpha * this->dGddof_.coeff(0);
	f.coeffRef(k) = this->b_ - kBeta * kBeta * this->G_;
};