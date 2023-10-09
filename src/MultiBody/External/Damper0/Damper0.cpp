#include "MultiBody/External/Damper0/Damper0.h"

Damper0::Damper0(const int body1, const double r,
	const Eigen::Vector3d& pos1, const Eigen::Vector3d& axis) :
	External(body1), r_(r), pos1_(pos1), axis_(axis)
{};

void Damper0::updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f) {
	
	Eigen::Vector3d vel1G = body[this->body1_]->getDof().segment<3>(0) +
		body[this->body1_]->getQuaternion().rotateVecG(body[this->body1_]->getWLocal().cross(this->pos1_)); // TBD: CHECK QUATERNIONS + SOR

	this->ext_ = this->r_ * vel1G.dot(this->axis_);

	f.segment<3>(this->body1_ * kDof) -= this->ext_ * this->axis_;
	f.segment<4>(this->body1_ * kDof + 3) -=
		2.0 * body[this->body1_]->getG().transpose() * body[this->body1_]->getQuaternion().rotateVecL(this->pos1_).cross(body[this->body1_]->getQuaternion().rotateVecL(this->ext_ * this->axis_));

};