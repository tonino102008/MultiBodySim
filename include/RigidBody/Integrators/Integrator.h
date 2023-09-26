#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_INTEGRATOR_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_INTEGRATOR_H_

#include "RigidBody/RigidBody.h"
#include "RigidBody/Constraints/Constraints.h"

#include <Eigen/Dense>
#include <vector>
#include <ostream>

class Integrator {

public:

	Integrator(const double timeStart, const double timeEnd,
		const double dt, const double timeActual,
		std::vector<std::reference_wrapper<RigidBody>> body,
		std::vector<std::reference_wrapper<Constraint>> constraint);

	double getTimeStart() const;

	double getTimeEnd() const;

	double getDt() const;

	double getTimeActual() const;

	int getNSteps() const;

	Eigen::MatrixXd getdofTimeHistory() const;

	Eigen::MatrixXd getMass() const;

	Eigen::VectorXd getF() const;

	friend std::ostream& operator<<(std::ostream& out, const Integrator& I);

	virtual void solve() = 0;

protected:

	const double timeStart_;

	const double timeEnd_;

	const double dt_;

	double timeActual_;

	const int nSteps_;

	Eigen::MatrixXd dofTimeHistory_;

	Eigen::MatrixXd M_;

	Eigen::VectorXd f_;

	std::vector<std::reference_wrapper<RigidBody>> body_;

	std::vector<std::reference_wrapper<Constraint>> constraint_;
	
};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_INTEGRATOR_H_
