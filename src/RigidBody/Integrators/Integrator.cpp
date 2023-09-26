#include "RigidBody/Integrators/Integrator.h"

Integrator::Integrator(const double timeStart, const double timeEnd,
	const double dt, const double timeActual,
	std::vector<std::reference_wrapper<RigidBody>> body,
	std::vector<std::reference_wrapper<Constraint>> constraint) :
	timeStart_(timeStart), timeEnd_(timeEnd),
	dt_(dt), timeActual_(timeActual),
	nSteps_((timeEnd - timeStart) / dt), body_(body), constraint_(constraint),
	dofTimeHistory_(Eigen::MatrixXd::Zero(body[0].get().getDof().rows() * body.size() + constraint.size(),
		static_cast<int>((timeEnd - timeStart) / dt + 1))),
	M_(Eigen::MatrixXd::Zero(body[0].get().getDof().rows() * body.size() + constraint.size(),
		body[0].get().getDof().rows()* body.size() + constraint.size())),
	f_(Eigen::VectorXd::Zero(body[0].get().getDof().rows() * body.size() + constraint.size()))
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

Eigen::MatrixXd Integrator::getdofTimeHistory() const {
	return this->dofTimeHistory_;
};

Eigen::MatrixXd Integrator::getMass() const {
	return this->M_;
};

Eigen::VectorXd Integrator::getF() const {
	return this->f_;
};

std::ostream& operator<<(std::ostream& out, const Integrator& I) {
	out << "Time vector: [" << I.timeStart_ << ":" << I.dt_ << ":" << I.timeEnd_ << "]s\n" 
		<< "Actual Time Step: " << I.timeActual_ << "s" << std::endl;
	out << "DOFs Time Evolution: \n" << I.dofTimeHistory_ << std::endl;
	return out;
};