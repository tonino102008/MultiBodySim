#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER_DAMPER_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER_DAMPER_H_

#include "MultiBody/External/External.h"

#include <Eigen/Dense>

class Damper : public External {

public:

	Damper(const int body1, const int body2, const double r,
		const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& axis);

	Eigen::VectorXi getBodyIndex() const;

	double getExt() const;

	void updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f);

private:

	const int body1_;

	const int body2_;

	double ext_;

	const double r_;

	const Eigen::Vector3d pos1_;

	const Eigen::Vector3d pos2_;

	const Eigen::Vector3d& axis_;
};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER_DAMPER_H_