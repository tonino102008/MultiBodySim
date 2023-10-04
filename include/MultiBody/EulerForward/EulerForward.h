#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EULERFORWARD_EULERFORWARD_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EULERFORWARD_EULERFORWARD_H_

#include "MultiBody/MultiBody.h"

class EulerForward : public MultiBody {

public:

	EulerForward(const double timeStart, const double timeEnd,
		const double dt, const double timeActual,
		std::vector<std::reference_wrapper<RigidBody>> body,
		std::vector<std::reference_wrapper<Constraint>> constraint,
		std::vector<std::reference_wrapper<External>> external);

	void solve();

	void print() const;

	void printToFile() const;

};

#endif //MULTIBODYSIM_INCLUDE_MULTIBODY_EULERFORWARD_EULERFORWARD_H_