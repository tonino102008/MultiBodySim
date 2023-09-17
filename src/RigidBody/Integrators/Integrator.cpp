#include "RigidBody/Integrators/Integrator.h"

Integrator::Integrator() :
	timeStart_(0), timeEnd_(0),
	dt_(0), timeActual_(0),
	xActual_(MatrixN(2,kSingleColumn)), xPrevious_(MatrixN(2,kSingleColumn))
{};

Integrator::Integrator(const double timeStart, const double timeEnd,
	const double dt, const double timeActual,
	const RigidBody& Body) :
	timeStart_(timeStart), timeEnd_(timeEnd),
	dt_(dt), timeActual_(timeActual),
	xActual_(Body.getDof()), xPrevious_(Body.getDof())
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

MatrixN Integrator::getXAct() const {
	return this->xActual_;
};

MatrixN Integrator::getXPrev() const {
	return this->xPrevious_;
};

std::ostream& operator<<(std::ostream& out, const Integrator& I) {
	out << "Time vector: [" << I.timeStart_ << ":" << I.dt_ << ":" << I.timeEnd_ << "]s\n" 
		<< "Actual Time Step: " << I.timeActual_ << "s" << std::endl;
	out << "Actual Vector: " << I.xActual_ << std::endl;
	out << "Previous Vector: " << I.xPrevious_;
	return out;
};