#include "MultiBody/External/Spring0/Spring0.h"

Spring0::Spring0(const int body1, const double k, const double x0,
	const Eigen::Vector3d& pos1, const Eigen::Vector3d& axis) :
	External(body1), k_(k), x0_(x0), pos1_(pos1), axis_(axis)
{};

void Spring0::updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f) {

	Eigen::Vector3d pos1G = body[this->body1_]->getDof().segment<3>(7) +
		(body[this->body1_]->getQuaternion() * Quaternion(0.0, this->pos1_) * body[this->body1_]->getQuaternion().conj()).getVector();

	this->ext_ = this->k_ * (this->x0_ + pos1G.dot(this->axis_));

	f.segment<3>(this->body1_ * kDof) -= this->ext_ * this->axis_;
	f.segment<4>(this->body1_ * kDof + 3) -=
		2.0 * body[this->body1_]->getG().transpose() * this->pos1_.cross(
			(body[this->body1_]->getQuaternion().conj() * Quaternion(0.0, this->ext_ * this->axis_) * body[this->body1_]->getQuaternion()).getVector());

};