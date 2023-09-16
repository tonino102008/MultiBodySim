#include "RigidBody/Integrators/Integrator.h"

Integrator::Integrator() :
	timeStart_(0), timeEnd_(0),
	dt_(0), timeActual_(0),
	xActual_(VectorN(2)), xPrevious_(VectorN(2))
{};

Integrator::Integrator(const double timeStart, const double timeEnd,
	const double dt, const double timeActual,
	const VectorN& xActual, const VectorN& xPrevious) :
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

VectorN Integrator::getXAct() const {
	return this->xActual_;
};

VectorN Integrator::getXPrev() const {
	return this->xPrevious_;
};

std::ostream& operator<<(std::ostream& out, const Integrator& I) {
	out << "Time vector: [" << I.timeStart_ << ":" << I.dt_ << ":" << I.timeEnd_ << "]s\n" 
		<< "Actual Time Step: " << I.timeActual_ << "s" << std::endl;
	out << "Actual Vector: ";
	for (int i = 0; i < I.xActual_.getSize(); i++)
		out << I.xActual_[i] << " ";
	out << "\nPrevious Vector: ";
	for (int i = 0; i < I.xPrevious_.getSize(); i++)
		out << I.xPrevious_[i] << " ";
	return out;
};