#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_

#include "MultiBody/RigidBody/RigidBody.h"

#include <Eigen/Dense>

class External {

public:

	External(const int body1);

	External(const int body1, const int body2);

	int getDof1() const;

	int getDof2() const;

	double getExt() const;

	virtual void updateExternal(const std::vector<std::unique_ptr<RigidBody>>& body, Eigen::VectorXd& f) = 0;

protected:

	const int body1_;

	const int body2_;

	double ext_;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_