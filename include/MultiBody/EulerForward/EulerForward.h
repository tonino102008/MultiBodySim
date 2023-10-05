#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_EULERFORWARD_EULERFORWARD_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_EULERFORWARD_EULERFORWARD_H_

#include "MultiBody/MultiBody.h"

class EulerForward : public MultiBody {

public:

	EulerForward(const double timeStart, const double timeEnd,
		const double dt, const double timeActual,
		const int nBody, const int nConstr, const int nExt);

	void solve();

	void print() const;

	void printToFile() const;

private:

	void solve0();

};

#endif //MULTIBODYSIM_INCLUDE_MULTIBODY_EULERFORWARD_EULERFORWARD_H_