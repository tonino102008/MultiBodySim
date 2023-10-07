#include "MultiBody/MultiBody.h"

#include <fstream>

MultiBody::MultiBody(const double timeStart, const double timeEnd,
	const double dt, const double timeActual,
	const int nBody, const int nConstr, const int nExt) :
	time_(std::make_unique<TimeSim>(timeStart, timeEnd, dt, timeActual)),
	body_(nBody), constraint_(nConstr), external_(nExt), integrator_(),
	dofTimeHistory_(Eigen::MatrixXd::Zero(kDof * nBody + nConstr,
		static_cast<int>((timeEnd - timeStart) / dt + 1))),
	M_(Eigen::MatrixXd::Zero(kDof* nBody + nConstr,	kDof* nBody + nConstr)),
	f_(Eigen::VectorXd::Zero(kDof* nBody + nConstr)),
	nBody_(nBody), nConstr_(nConstr), nExt_(nExt)
{};

Eigen::MatrixXd MultiBody::getdofTimeHistory() const {
	return this->dofTimeHistory_;
};

Eigen::MatrixXd MultiBody::getMass() const {
	return this->M_;
};

Eigen::VectorXd MultiBody::getF() const {
	return this->f_;
};

void MultiBody::setBody(const RigidBody& body, const int i) {
	this->body_[i] = std::make_shared<RigidBody>(body);
};

void MultiBody::solve() {
	this->integrator_->solve(this->dofTimeHistory_, this->M_, this->f_,
		this->body_, this->constraint_, this->external_, this->time_);
};

void MultiBody::printToFile() const {
	std::ofstream myfile;
	myfile.open("output.txt");
	myfile << this->dofTimeHistory_;
	myfile.close();
};

std::ostream& operator<<(std::ostream& out, const MultiBody& I) {
	out << "Time vector: [" << I.time_->getTimeStart() << ":" << I.time_->getDt() << ":" << I.time_->getTimeEnd() << "]s\n"
		<< "Actual Time Step: " << I.time_->getTimeActual() << "s" << std::endl;
	out << "DOFs Time Evolution: \n" << I.dofTimeHistory_ << std::endl;
	return out;
};