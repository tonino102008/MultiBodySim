#include "MultiBody/External/Damper0/Damper0.h"

Damper0::Damper0(const int body1, const double r, const Eigen::Vector3d& pos1,
	const Eigen::Vector3d& pos2, const Eigen::Vector3d& axis) :
	body1_(body1), r_(r), pos1_(pos1), pos2_(pos2), axis_(axis), ext_(0.0)
{};

Eigen::VectorXi Damper0::getBodyIndex() const {
	return Eigen::VectorXi(this->body1_);
};

double Damper0::getExt() const {
	return this->ext_;
};

void Damper0::updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f) {
	
	Eigen::Vector3d vel1G = body[this->body1_]->getDof().segment<3>(0) +
		body[this->body1_]->getQuaternion().rotateVecG(body[this->body1_]->getWLocal().cross(this->pos1_)); // TBD: CHECK QUATERNIONS + SOR

	this->ext_ = this->r_ * (vel1G.dot(this->axis_) - this->pos2_.dot(this->axis_));

	f.segment<3>(this->body1_ * kDof) -= this->ext_ * this->axis_;
	f.segment<4>(this->body1_ * kDof + 3) -=
		2.0 * body[this->body1_]->getG().transpose() * body[this->body1_]->getQuaternion().rotateVecL(this->pos1_).cross(body[this->body1_]->getQuaternion().rotateVecL(this->ext_ * this->axis_));

};