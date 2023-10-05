#include "MultiBody/EulerForward/EulerForward.h"

#include <iostream>
#include <fstream>

EulerForward::EulerForward(const double timeStart, const double timeEnd,
	const double dt, const double timeActual, 
	const int nBody, const int nConstr, const int nExt) :
	MultiBody(timeStart, timeEnd, dt, timeActual, nBody, nConstr, nExt)
{};

void EulerForward::solve0() {
	for (int j = 0; j < this->nBody_; j++) {
		int k = j * kDof;
		this->dofTimeHistory_.block<kDof, 1>(k, 0) = this->body_[j]->getDof();
		this->M_.block<kDof, kDof>(k, k) = this->body_[j]->getMass();
		this->f_.segment<kDof>(k) = this->body_[j]->getF();
		//this->print();
	}
	for (int j = 0; j < this->nConstr_; j++) {
		int k = j + this->nBody_ * kDof;
		this->constraint_[j]->updateConstraint(this->dofTimeHistory_.col(0), this->M_, this->f_, k);
		//std::cout << "G: " << this->constraint_[j].get().getG() << std::endl;
	}
	for (int j = 0; j < this->nExt_; j++) {
		this->external_[j]->updateExternal(this->dofTimeHistory_.col(0), this->f_);
		//std::cout << "Ext: " << this->external_[j].get().getExt() << std::endl;
	}
};

void EulerForward::solve() {
	this->solve0();
	for (int i = 0; i < this->time_->getNSteps(); i++) {
		this->time_->step();
		Eigen::VectorXd dofp = this->M_.lu().solve(this->f_);
		Eigen::VectorXd dof = this->dofTimeHistory_.col(i) + dofp * this->time_->getDt();
		this->dofTimeHistory_.col(i + 1) = dof;
		for (int j = 0; j < this->nBody_; j++) {
			int k = j * kDof;
			this->body_[j]->updateBody(dof, this->M_, this->f_, k);
			//this->print();
		}
		for (int j = 0; j < this->nConstr_; j++) {
			int k = j + this->nBody_ * kDof;
			this->constraint_[j]->updateConstraint(dof, this->M_, this->f_, k);
			//std::cout << "G: " << this->constraint_[j].get().getG() << std::endl;
		}
		for (int j = 0; j < this->nExt_; j++) {
			this->external_[j]->updateExternal(dof, this->f_);
			//std::cout << "Ext: " << this->external_[j].get().getExt() << std::endl;
		}
	}
};

void EulerForward::print() const {
	std::cout << "Actual Time Step: " << this->time_->getTimeActual() << "s" << std::endl;
	for (int j = 0; j < this->body_.size(); j++) {
		std::cout << "Dof: \n" << this->body_[j]->getDof() << std::endl;
		Quaternion q(this->body_[j]->getDof().coeff(10), this->body_[j]->getDof().segment<3>(11));
		std::cout << "Quaternion Constraint: " << 1.0 - q.norm() << std::endl;
	}
}

void EulerForward::printToFile() const {
	std::ofstream myfile;
	myfile.open("output.txt");
	myfile << this->getdofTimeHistory();
	myfile.close();
};