#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER0_DAMPER0_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER0_DAMPER0_H_

#include "MultiBody/External/External.h"

#include <Eigen/Dense>

class Damper0 : public External {

public:

	Damper0(const int body1, const double r,
		const Eigen::Vector3d& pos1, const Eigen::Vector3d& axis);

	void updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f);

private:

	const double r_;

	const Eigen::Vector3d& pos1_;
	
	const Eigen::Vector3d& axis_;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER0_DAMPER0_H_