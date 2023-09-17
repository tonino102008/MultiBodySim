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
		const double dt, const double timeActual,
		const RigidBody& Body);

	double getTimeStart() const;

	double getTimeEnd() const;

	double getDt() const;

	double getTimeActual() const;

	MatrixN getXAct() const;

	MatrixN getXPrev() const;

	friend std::ostream& operator<<(std::ostream& out, const Integrator& I);

private:

	double timeStart_;

	double timeEnd_;

	double dt_;

	double timeActual_;

	MatrixN xActual_;

	MatrixN xPrevious_;
	
};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_INTEGRATOR_H_
