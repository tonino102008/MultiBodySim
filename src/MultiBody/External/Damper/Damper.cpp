#include "MultiBody/External/Damper/Damper.h"

#include <iostream>

Damper::Damper(const int body1, const int body2, const double r,
	const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& axis) :
	body1_(body1), body2_(body2), r_(r), pos1_(pos1), pos2_(pos2), axis_(axis), ext_(0.0)
{};

Eigen::VectorXi Damper::getBodyIndex() const {
	return Eigen::VectorXi(this->body1_, this->body2_);
};

double Damper::getExt() const {
	return this->ext_;
};

void Damper::updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f) {

	Eigen::Vector3d vel1G = body[this->body1_]->getDof().segment<3>(0) +
		body[this->body1_]->getQuaternion().rotateVecG(body[this->body1_]->getWLocal().cross(this->pos1_));

	Eigen::Vector3d vel2G = body[this->body2_]->getDof().segment<3>(0) +
		body[this->body2_]->getQuaternion().rotateVecG(body[this->body2_]->getWLocal().cross(this->pos2_));

	this->ext_ = this->r_ * (vel2G.dot(this->axis_) - vel1G.dot(this->axis_));

	f.segment<3>(this->body1_ * kDof) += this->ext_ * this->axis_;
	f.segment<4>(this->body1_ * kDof + 3) +=
		2.0 * body[this->body1_]->getG().transpose() * body[this->body1_]->getQuaternion().rotateVecL(this->pos1_).cross(body[this->body1_]->getQuaternion().rotateVecL(this->ext_ * this->axis_));

	f.segment<3>(this->body2_ * kDof) -= this->ext_ * this->axis_;
	f.segment<4>(this->body2_ * kDof + 3) -=
		2.0 * body[this->body2_]->getG().transpose() * body[this->body2_]->getQuaternion().rotateVecL(this->pos2_).cross(body[this->body2_]->getQuaternion().rotateVecL(this->ext_ * this->axis_));

};