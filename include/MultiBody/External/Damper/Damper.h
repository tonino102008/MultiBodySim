#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER_DAMPER_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER_DAMPER_H_

#include "MultiBody/External/External.h"

#include <Eigen/Dense>

class Damper : public External {

public:

	Damper(const int dof1, const double r);

	Damper(const int dof1, const int dof2, const double r);

	void updateExt(const Eigen::VectorXd& dof);

private:

	const double r_;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_DAMPER_DAMPER_H_