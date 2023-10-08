#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER_DAMPER_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER_DAMPER_H_

#include "MultiBody/External/External.h"

#include <Eigen/Dense>

class Damper : public External {

public:

	Damper(const int dof1, const int dof2, const double r,
		const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& axis);

	void updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f);

private:

	const double r_;

	const Eigen::Vector3d pos1_;

	const Eigen::Vector3d pos2_;

	const Eigen::Vector3d& axis_;
};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER_DAMPER_H_