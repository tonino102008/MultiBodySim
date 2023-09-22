#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_INTEGRATOR_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_INTEGRATOR_H_

#include "RigidBody/MatrixN/MatrixN.h"
#include "RigidBody/RigidBody.h"

#include <vector>
#include <ostream>

class Integrator {

public:

	Integrator();

	Integrator(const double timeStart, const double timeEnd,
		const double dt, const double timeActual, std::vector<std::reference_wrapper<RigidBody>> body);

	double getTimeStart() const;

	double getTimeEnd() const;

	double getDt() const;

	double getTimeActual() const;

	int getNSteps() const;

	MatrixN getdofTimeHistory() const;

	MatrixN getMass() const;

	MatrixN getF() const;

	friend std::ostream& operator<<(std::ostream& out, const Integrator& I);

	virtual void solve() = 0;

protected:

	const double timeStart_;

	const double timeEnd_;

	const double dt_;

	double timeActual_;

	const int nSteps_;

	MatrixN dofTimeHistory_;

	MatrixN M_;

	MatrixN f_;

	std::vector<std::reference_wrapper<RigidBody>> body_;
	
};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_INTEGRATOR_H_
