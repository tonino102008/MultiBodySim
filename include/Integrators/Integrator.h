#ifndef MULTIBODYSIM_INCLUDE_INTEGRATORS_INTEGRATOR_H_
#define MULTIBODYSIM_INCLUDE_INTEGRATORS_INTEGRATOR_H_

#include <vector>
#include <ostream>

class Integrator {

public:

	Integrator();

	Integrator(const double timeStart, const double timeEnd,
		const double dt, const double timeActual,
		const std::vector<double>& xActual, const std::vector<double>& xPrevious);

	double getTimeStart() const;

	double getTimeEnd() const;

	double getDt() const;

	double getTimeActual() const;

	std::vector<double> getXAct() const;

	std::vector<double> getXPrev() const;

	friend std::ostream& operator<<(std::ostream& out, const Integrator& I);

private:

	double timeStart_;

	double timeEnd_;

	double dt_;

	double timeActual_;

	std::vector<double> xActual_;

	std::vector<double> xPrevious_;
	
};

#endif // MULTIBODYSIM_INCLUDE_INTEGRATORS_INTEGRATOR_H_
