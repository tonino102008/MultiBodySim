#include "Integrator.h"

Integrator::Integrator() :
	timeStart(0), timeEnd(0),
	dt(0), timeActual(0),
	xAct(std::vector<double>{ 0, 0 }), xPrev(std::vector<double>{ 0, 0 })
{};

Integrator::Integrator(const double timeStart, const double timeEnd,
	const double dt, const double timeActual,
	const std::vector<double>& xAct, const std::vector<double>& xPrev) :
	timeStart(timeStart), timeEnd(timeEnd),
	dt(dt), timeActual(timeActual),
	xAct(xAct),	xPrev(xPrev) 
{};

double Integrator::getTimeStart() const {
	return this->timeStart;
};

double Integrator::getTimeEnd() const {
	return this->timeEnd;
};

double Integrator::getDt() const {
	return this->dt;
};

double Integrator::getTimeActual() const {
	return this->timeActual;
};

std::vector<double> Integrator::getXAct() const {
	return this->xAct;
};

std::vector<double> Integrator::getXPrev() const {
	return this->xPrev;
};