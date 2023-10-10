#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_

#include "MultiBody/RigidBody/RigidBody.h"

#include <Eigen/Dense>

class External {

public:

	External();

	virtual Eigen::VectorXi getBodyIndex() const = 0;

	virtual double getExt() const = 0;

	virtual void updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f) = 0;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_