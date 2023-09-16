#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_INTEGRATOR_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_INTEGRATOR_H_

#include "RigidBody/VectorN/VectorN.h"

#include <vector>
#include <ostream>

class Integrator {

public:

	Integrator();

	Integrator(const double timeStart, const double timeEnd,
		const double dt, const double timeActual,
		const VectorN& xActual, const VectorN& xPrevious);

	double getTimeStart() const;

	double getTimeEnd() const;

	double getDt() const;

	double getTimeActual() const;

	VectorN getXAct() const;

	VectorN getXPrev() const;

	friend std::ostream& operator<<(std::ostream& out, const Integrator& I);

private:

	double timeStart_;

	double timeEnd_;

	double dt_;

	double timeActual_;

	VectorN xActual_;

	VectorN xPrevious_;
	
};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_INTEGRATOR_H_
