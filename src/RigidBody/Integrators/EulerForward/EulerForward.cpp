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
		this->dofTimeHistory_.fill(j * 15, 0, this->body_[j].get().getDof());
		this->M_.fill(j * 15, j * 15, this->body_[j].get().getMass());
		this->f_.fill(j * 15, 0, this->body_[j].get().getF());
	}
	for (int i = 0; i < this->nSteps_; i++) {
		this->timeActual_ += this->dt_;
		MatrixN dofp = (this->M_ + MatrixN(this->M_.getSize()[0], 100.0 * DBL_EPSILON)) / this->f_;
		MatrixN dof = this->dofTimeHistory_.slice(0, this->dofTimeHistory_.getSize()[0], i, i + 1) + (dofp * this->dt_);
		for (int j = 0; j < this->body_.size(); j++) {
			int k = j * 15;
			this->body_[j].get().updateXGp(dof.slice(k, k + 3, 0, 1));
			this->body_[j].get().updateQp(Quaternion(dof.slice(k + 3, k + 4, 0, 1)[0][0], dof.slice(k + 4, k + 7, 0, 1)));
			this->body_[j].get().updateXG(dof.slice(k + 7, k + 10, 0, 1));
			this->body_[j].get().updateQ(Quaternion(dof.slice(k + 10, k + 11, 0, 1)[0][0], dof.slice(k + 11, k + 14, 0, 1)));
			this->body_[j].get().updateDof();
			this->body_[j].get().updateMass();
			this->body_[j].get().updateF();
			this->dofTimeHistory_.fill(k, i + 1, this->body_[j].get().getDof());
			this->M_.fill(k, k, this->body_[j].get().getMass());
			this->f_.fill(k, 0, this->body_[j].get().getF());
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