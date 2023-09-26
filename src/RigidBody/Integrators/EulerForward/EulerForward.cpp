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
		int k = j * 15;
		this->dofTimeHistory_.block<15, 1>(k, 0) = this->body_[j].get().getDof();
		this->M_.block<15, 15>(k, k) = this->body_[j].get().getMass();
		this->f_.segment<15>(k) = this->body_[j].get().getF();
		//this->print();
	}
	for (int j = 0; j < nConstr; j++) {
		int n = j + nBody * 15;
		this->constraint_[j].get().updateConstraint(this->dofTimeHistory_.col(0));
		const double dof1 = this->constraint_[j].get().getDof1();
		const double dof2 = this->constraint_[j].get().getDof2();
		const double dof1p = dof1- 7;
		const double dof2p = dof2 - 7;
		//this->dofTimeHistory_.coeffRef(n, 0) = 0.0;
		this->M_.coeffRef(n, dof1) = this->constraint_[j].get().getDGDDof().coeff(0);
		this->M_.coeffRef(dof1, n) = this->constraint_[j].get().getDGDDof().coeff(0);
		this->M_.coeffRef(n, dof2) = this->constraint_[j].get().getDGDDof().coeff(1);
		this->M_.coeffRef(dof2, n) = this->constraint_[j].get().getDGDDof().coeff(1);
		this->M_.coeffRef(n, dof1p) = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof().coeff(0);
		this->M_.coeffRef(dof1p, n) = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof().coeff(0);
		this->M_.coeffRef(n, dof2p) = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof().coeff(1);
		this->M_.coeffRef(dof2p, n) = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof().coeff(1);
		this->f_.coeffRef(n) = this->constraint_[j].get().getB()
			- kBeta * kBeta * this->constraint_[j].get().getG();
		std::cout << "G: " << this->constraint_[j].get().getG() << std::endl;
	}
	for (int i = 0; i < this->nSteps_; i++) {
		this->timeActual_ += this->dt_;
		Eigen::VectorXd dofp = this->M_.lu().solve(this->f_);
		Eigen::VectorXd dof = this->dofTimeHistory_.col(i) + dofp * this->dt_;
		this->dofTimeHistory_.col(i + 1) = dof;
		for (int j = 0; j < nBody; j++) {
			int k = j * 15;
			this->body_[j].get().updateXGp(dof.segment<3>(k));
			this->body_[j].get().updateQp(Quaternion(dof.coeff(k + 3), dof.segment<3>(k + 4)));
			this->body_[j].get().updateXG(dof.segment<3>(k + 7));
			this->body_[j].get().updateQ(Quaternion(dof.coeff(k + 10), dof.segment<3>(k + 11)));
			this->body_[j].get().updateLambda(dof.coeff(k + 14));
			this->body_[j].get().updateDof();
			this->body_[j].get().updateMass();
			this->body_[j].get().updateF();
			this->M_.block<15, 15>(k, k) = this->body_[j].get().getMass();
			this->f_.segment<15>(k) = this->body_[j].get().getF();
			//this->print();
		}
		for (int j = 0; j < nConstr; j++) {
			int n = j + nBody * 15;
			this->constraint_[j].get().updateConstraint(dof);
			const double dof1 = this->constraint_[j].get().getDof1();
			const double dof2 = this->constraint_[j].get().getDof2();
			const double dof1p = dof1 - 7;
			const double dof2p = dof2 - 7;
			this->M_.coeffRef(n, dof1) = this->constraint_[j].get().getDGDDof().coeff(0);
			this->M_.coeffRef(dof1, n) = this->constraint_[j].get().getDGDDof().coeff(0);
			this->M_.coeffRef(n, dof2) = this->constraint_[j].get().getDGDDof().coeff(1);
			this->M_.coeffRef(dof2, n) = this->constraint_[j].get().getDGDDof().coeff(1);
			this->M_.coeffRef(n, dof1p) = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof().coeff(0);
			this->M_.coeffRef(dof1p, n) = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof().coeff(0);
			this->M_.coeffRef(n, dof2p) = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof().coeff(1);
			this->M_.coeffRef(dof2p, n) = 2.0 * kAlpha * this->constraint_[j].get().getDGDDof().coeff(1);
			this->f_.coeffRef(n) = this->constraint_[j].get().getB()
				- kBeta * kBeta * this->constraint_[j].get().getG();
			std::cout << "G: " << this->constraint_[j].get().getG() << std::endl;
		}
	}
};

void EulerForward::print() const {
	std::cout << "Actual Time Step: " << this->timeActual_ << "s" << std::endl;
	for (int j = 0; j < this->body_.size(); j++) {
		std::cout << "Dof: \n" << this->body_[j].get().getDof() << std::endl;
		Quaternion q(this->body_[j].get().getDof().coeff(10), this->body_[j].get().getDof().segment<3>(11));
		std::cout << "Quaternion Constraint: " << 1.0 - q.norm() << std::endl;
	}
}

void EulerForward::printToFile() const {
	std::ofstream myfile;
	myfile.open("output.txt");
	myfile << this->getdofTimeHistory();
	myfile.close();
};