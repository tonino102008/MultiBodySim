#include "MultiBody/External/Spring0/Spring0.h"

Spring0::Spring0(const int body1, const double k, const double x0, const Eigen::Vector3d& pos1,
	const Eigen::Vector3d& pos2, const Eigen::Vector3d& axis) :
	body1_(body1), k_(k), x0_(x0), pos1_(pos1), pos2_(pos2), axis_(axis), ext_(0.0)
{};

Eigen::VectorXi Spring0::getBodyIndex() const {
	return Eigen::VectorXi(this->body1_);
};

double Spring0::getExt() const {
	return this->ext_;
};

void Spring0::updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f) {

	Eigen::Vector3d pos1G = body[this->body1_]->getDof().segment<3>(7) + body[this->body1_]->getQuaternion().rotateVecG(this->pos1_);

	this->ext_ = this->k_ * (this->x0_ + pos1G.dot(this->axis_) - this->pos2_.dot(this->axis_));

	f.segment<3>(this->body1_ * kDof) -= this->ext_ * this->axis_;
	f.segment<4>(this->body1_ * kDof + 3) -=
		2.0 * body[this->body1_]->getG().transpose() * body[this->body1_]->getQuaternion().rotateVecL(this->pos1_).cross(body[this->body1_]->getQuaternion().rotateVecL(this->ext_ * this->axis_));

};