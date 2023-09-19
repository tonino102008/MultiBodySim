#include "RigidBody/Integrators/EulerForward/EulerForward.h"

#include <iostream>

EulerForward::EulerForward(const double timeStart, const double timeEnd,
	const double dt, const double timeActual, RigidBody& body) :
	Integrator(timeStart, timeEnd, dt, timeActual, body)
{};

void EulerForward::solve() {
	this->print();
	this->dofTimeHistory_.fill(0, 0, this->body_.getDof());
	for (int i = 0; i < this->nSteps_; i++) {
		this->timeActual_ += this->dt_;
		MatrixN dofp = (this->body_.getMass() + MatrixN(15, 100.0 * DBL_EPSILON)) / this->body_.getF();
		MatrixN dof = this->body_.getDof() + (dofp * this->dt_);
		this->body_.updateXGp(dof.slice(0, 3, 0, 1));
		this->body_.updateQp(Quaternion(dof.slice(3, 4, 0, 1)[0][0], dof.slice(4, 7, 0, 1)));
		this->body_.updateXG(dof.slice(7, 10, 0, 1));
		this->body_.updateQ(Quaternion(dof.slice(10, 11, 0, 1)[0][0], dof.slice(11, 14, 0, 1)));
		this->body_.updateDof();
		this->body_.updateMass();
		this->body_.updateF();
		this->dofTimeHistory_.fill(0, i + 1, this->body_.getDof());
		this->print();
	}
};

void EulerForward::print() const {
	std::cout << "Actual Time Step: " << this->timeActual_ << "s" << std::endl;
	std::cout << "Dof: \n" << this->body_.getDof() << std::endl;
}