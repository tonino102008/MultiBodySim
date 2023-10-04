#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_SPRING_SPRING_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_SPRING_SPRING_H_

#include "MultiBody/External/External.h"

#include <Eigen/Dense>

class Spring : public External {

public:

	Spring(const int dof1, const double k, const double x0);

	Spring(const int dof1, const int dof2, const double k, const double x0);

	void updateExt(const Eigen::VectorXd& dof);

private:

	const double k_;

	const double x0_;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_SPRING_SPRING_H_