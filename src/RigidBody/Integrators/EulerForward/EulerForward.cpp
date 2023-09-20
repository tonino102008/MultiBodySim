#include "RigidBody/Integrators/EulerForward/EulerForward.h"

#include <iostream>
#include <fstream>

EulerForward::EulerForward(const double timeStart, const double timeEnd,
	const double dt, const double timeActual, std::vector<std::reference_wrapper<RigidBody>> body) :
	Integrator(timeStart, timeEnd, dt, timeActual, body)
{};

void EulerForward::solve() {
	for (int j = 0; j < this->body_.size(); j++) {
		//this->print();
		this->dofTimeHistory_.fill(0, 0, this->body_[j].get().getDof());
	}
	for (int i = 0; i < this->nSteps_; i++) {
		this->timeActual_ += this->dt_;
		for (int j = 0; j < this->body_.size(); j++) {
			MatrixN dofp = (this->body_[j].get().getMass() + MatrixN(15, 100.0 * DBL_EPSILON)) / this->body_[j].get().getF();
			MatrixN dof = body_[j].get().getDof() + (dofp * this->dt_);
			this->body_[j].get().updateXGp(dof.slice(0, 3, 0, 1));
			this->body_[j].get().updateQp(Quaternion(dof.slice(3, 4, 0, 1)[0][0], dof.slice(4, 7, 0, 1)));
			this->body_[j].get().updateXG(dof.slice(7, 10, 0, 1));
			this->body_[j].get().updateQ(Quaternion(dof.slice(10, 11, 0, 1)[0][0], dof.slice(11, 14, 0, 1)));
			this->body_[j].get().updateDof();
			this->body_[j].get().updateMass();
			this->body_[j].get().updateF();
			this->dofTimeHistory_.fill(0, i + 1, this->body_[j].get().getDof()); // TODO: modify 0 in j * 15
			//this->print();
		}
	}
};

void EulerForward::print() const {
	std::cout << "Actual Time Step: " << this->timeActual_ << "s" << std::endl;
	for (int j = 0; j < this->body_.size(); j++)
		std::cout << "Dof: \n" << this->body_[j].get().getDof() << std::endl;
}

void EulerForward::printToFile() const {
	std::ofstream myfile;
	myfile.open("output.txt");
	myfile << this->getdofTimeHistory();
	myfile.close();
};