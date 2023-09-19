#include "RigidBody/Integrators/Integrator.h"

Integrator::Integrator(const double timeStart, const double timeEnd,
	const double dt, const double timeActual, RigidBody& body) :
	timeStart_(timeStart), timeEnd_(timeEnd),
	dt_(dt), timeActual_(timeActual),
	nSteps_((timeEnd - timeStart) / dt), body_(body),
	dofTimeHistory_(MatrixN(body.getDof().getSize()[0], (timeEnd - timeStart + dt)/dt, 0.0))
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

int Integrator::getNSteps() const {
	return this->nSteps_;
};

MatrixN Integrator::getdofTimeHistory() const {
	return this->dofTimeHistory_;
};

std::ostream& operator<<(std::ostream& out, const Integrator& I) {
	out << "Time vector: [" << I.timeStart_ << ":" << I.dt_ << ":" << I.timeEnd_ << "]s\n" 
		<< "Actual Time Step: " << I.timeActual_ << "s" << std::endl;
	out << "DOFs Time Evolution: \n" << I.dofTimeHistory_ << std::endl;
	return out;
};