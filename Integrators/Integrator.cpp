#include "Integrator.h"

Integrator::Integrator() :
	timeStart_(0), timeEnd_(0),
	dt_(0), timeActual_(0),
	xActual_(std::vector<double>{ 0, 0 }), xPrevious_(std::vector<double>{ 0, 0 })
{};

Integrator::Integrator(const double timeStart, const double timeEnd,
	const double dt, const double timeActual,
	const std::vector<double>& xActual, const std::vector<double>& xPrevious) :
	timeStart_(timeStart), timeEnd_(timeEnd),
	dt_(dt), timeActual_(timeActual),
	xActual_(xActual), xPrevious_(xPrevious)
{};

double Integrator::getTimeStart() const {
	return this->timeStart_;
};

double Integrator::getTimeEnd() const {
	return this->timeEnd_;
};

double Integrator::getDt() const {
	return this->dt_;
};

double Integrator::getTimeActual() const {
	return this->timeActual_;
};

std::vector<double> Integrator::getXAct() const {
	return this->xActual_;
};

std::vector<double> Integrator::getXPrev() const {
	return this->xPrevious_;
};