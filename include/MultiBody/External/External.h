#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_

#include <Eigen/Dense>

class External {

public:

	External(const int dof1);

	External(const int dof1, const int dof2);

	int getDof1() const;

	int getDof2() const;

	double getExt() const;

	virtual void updateExt(const Eigen::VectorXd& dof) = 0;

protected:

	const int dof1_;

	const int dof2_;

	double ext_;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_EXTERNAL_EXTERNAL_H_