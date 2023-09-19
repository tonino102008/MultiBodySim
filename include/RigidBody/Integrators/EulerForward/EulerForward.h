#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_EULERFORWARD_EULERFORWARD_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_EULERFORWARD_EULERFORWARD_H_

#include "RigidBody/Integrators/Integrator.h"

class EulerForward : public Integrator {

public:

	EulerForward(const double timeStart, const double timeEnd,
		const double dt, const double timeActual,
		RigidBody& body);

	void solve();

	void print() const;

};

#endif //MULTIBODYSIM_INCLUDE_RIGIDBODY_INTEGRATORS_EULERFORWARD_EULERFORWARD_H_