#ifndef Integrator_h
#define Integrator_h

#include <vector>

class Integrator {

private:

	double timeStart;

	double timeEnd;

	double dt;

	double timeActual;

	std::vector<double> xAct;

	std::vector<double> xPrev;

public:

	Integrator();

	Integrator(const double timeStart, const double timeEnd,
		const double dt, const double timeActual,
		const std::vector<double>& xAct, const std::vector<double>& xPrev);

	double getTimeStart() const;

	double getTimeEnd() const;

	double getDt() const;

	double getTimeActual() const;

	std::vector<double> getXAct() const;

	std::vector<double> getXPrev() const;
	
};

#endif
