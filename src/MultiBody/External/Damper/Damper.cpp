#include "MultiBody/External/Damper/Damper.h"

Damper::Damper(const int dof1, const int dof2, const double r,
	const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& axis) :
	External(dof1, dof2), r_(r), pos1_(pos1), pos2_(pos2), axis_(axis)
{};

void Damper::updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f) {

	Eigen::Vector3d pos1G = body[this->body1_]->getDof().segment<3>(0) +
		body[this->body1_]->getE() * this->pos1_; // TBD: CHECK QUATERNIONS + SOR

	Eigen::Vector3d pos2G = body[this->body2_]->getDof().segment<3>(0) +
		body[this->body2_]->getE() * this->pos2_;

	this->ext_ = this->r_ * (pos2G.dot(this->axis_) - pos1G.dot(this->axis_));

	f.segment<3>(this->body1_ * kDof) += this->ext_ * this->axis_;
	f.segment<4>(this->body1_ * kDof + 3) +=
		2.0 * body[this->body1_]->getG().transpose() * this->pos1_.cross(this->ext_ * this->axis_);

	f.segment<3>(this->body2_ * kDof) -= this->ext_ * this->axis_;
	f.segment<4>(this->body2_ * kDof + 3) -=
		2.0 * body[this->body2_]->getG().transpose() * this->pos2_.cross(this->ext_ * this->axis_);

};