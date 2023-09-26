#include "MultiBody/MultiBody.h"

MultiBody::MultiBody(const double timeStart, const double timeEnd,
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

double MultiBody::getTimeStart() const {
	return this->timeStart_;
};

double MultiBody::getTimeEnd() const {
	return this->timeEnd_;
};

double MultiBody::getDt() const {
	return this->dt_;
};

double MultiBody::getTimeActual() const {
	return this->timeActual_;
};

int MultiBody::getNSteps() const {
	return this->nSteps_;
};

Eigen::MatrixXd MultiBody::getdofTimeHistory() const {
	return this->dofTimeHistory_;
};

Eigen::MatrixXd MultiBody::getMass() const {
	return this->M_;
};

Eigen::VectorXd MultiBody::getF() const {
	return this->f_;
};

std::ostream& operator<<(std::ostream& out, const MultiBody& I) {
	out << "Time vector: [" << I.timeStart_ << ":" << I.dt_ << ":" << I.timeEnd_ << "]s\n" 
		<< "Actual Time Step: " << I.timeActual_ << "s" << std::endl;
	out << "DOFs Time Evolution: \n" << I.dofTimeHistory_ << std::endl;
	return out;
};