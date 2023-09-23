#include "RigidBody/Integrators/EulerForward/EulerForward.h"

#include <iostream>
#include <fstream>

EulerForward::EulerForward(const double timeStart, const double timeEnd,
	const double dt, const double timeActual, 
	std::vector<std::reference_wrapper<RigidBody>> body,
	std::vector<std::reference_wrapper<Constraint>> constraint) :
	Integrator(timeStart, timeEnd, dt, timeActual, body, constraint)
{};

void EulerForward::solve() {
	const int nBody = this->body_.size();
	const int nConstr = this->constraint_.size();
	for (int j = 0; j < nBody; j++) {
		//this->print();
		this->dofTimeHistory_.fill(j * 15, 0, this->body_[j].get().getDof());
		this->M_.fill(j * 15, j * 15, this->body_[j].get().getMass());
		this->f_.fill(j * 15, 0, this->body_[j].get().getF());
	}
	for (int j = 0; j < nConstr; j++) {
		this->constraint_[j].get().updateConstraint(this->dofTimeHistory_.slice(0, this->dofTimeHistory_.getSize()[0], 0, 1));
		const double dof1 = this->constraint_[j].get().getDof1();
		const double dof2 = this->constraint_[j].get().getDof2();
		const double dof1p = dof1- 7;
		const double dof2p = dof2 - 7;
		this->dofTimeHistory_[j + nBody * 15][0] = 0.0;
		this->M_[j + nBody * 15][dof1] = this->constraint_[j].get().getDGDDof()[0][0];
		this->M_[dof1][j + nBody * 15] = this->constraint_[j].get().getDGDDof()[0][0];
		this->M_[j + nBody * 15][dof2] = this->constraint_[j].get().getDGDDof()[1][0];
		this->M_[dof2][j + nBody * 15] = this->constraint_[j].get().getDGDDof()[1][0];
		this->M_[j + nBody * 15][dof1p] = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof()[0][0];
		this->M_[dof1p][j + nBody * 15] = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof()[0][0];
		this->M_[j + nBody * 15][dof2p] = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof()[1][0];
		this->M_[dof2p][j + nBody * 15] = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof()[1][0];
		this->f_[j + nBody * 15][0] = this->constraint_[j].get().getB()
			- kBeta * kBeta * this->constraint_[j].get().getG();
		//std::cout << "G: " << this->constraint_[j].get().getG() << std::endl;
	}
	for (int i = 0; i < this->nSteps_; i++) {
		this->timeActual_ += this->dt_;
		MatrixN dofp = (this->M_ + MatrixN(this->M_.getSize()[0], 100.0 * DBL_EPSILON)) / this->f_;
		MatrixN dof = this->dofTimeHistory_.slice(0, this->dofTimeHistory_.getSize()[0], i, i + 1) + (dofp * this->dt_);
		for (int j = 0; j < nBody; j++) {
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
		for (int j = 0; j < nConstr; j++) {
			this->constraint_[j].get().updateConstraint(dof);
			const double dof1 = this->constraint_[j].get().getDof1();
			const double dof2 = this->constraint_[j].get().getDof2();
			const double dof1p = dof1 - 7;
			const double dof2p = dof2 - 7;
			this->M_[j + nBody * 15][dof1] = this->constraint_[j].get().getDGDDof()[0][0];
			this->M_[dof1][j + nBody * 15] = this->constraint_[j].get().getDGDDof()[0][0];
			this->M_[j + nBody * 15][dof2] = this->constraint_[j].get().getDGDDof()[1][0];
			this->M_[dof2][j + nBody * 15] = this->constraint_[j].get().getDGDDof()[1][0];
			this->M_[j + nBody * 15][dof1p] = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof()[0][0];
			this->M_[dof1p][j + nBody * 15] = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof()[0][0];
			this->M_[j + nBody * 15][dof2p] = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof()[1][0];
			this->M_[dof2p][j + nBody * 15] = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof()[1][0];
			this->f_[j + nBody * 15][0] = this->constraint_[j].get().getB()
				- kBeta * kBeta * this->constraint_[j].get().getG();
			//std::cout << "G: " << this->constraint_[j].get().getG() << std::endl;
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