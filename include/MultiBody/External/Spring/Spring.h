#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_SPRING_SPRING_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_SPRING_SPRING_H_

#include "MultiBody/External/External.h"

#include <Eigen/Dense>

class Spring : public External {

public:

	Spring(const int body1, const int body2, const double k, const double x0,
		const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& axis);

	void updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f);

private:

	const double k_;

	const double x0_;

	const Eigen::Vector3d pos1_;

	const Eigen::Vector3d pos2_;

	const Eigen::Vector3d axis_;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_SPRING_SPRING_H_