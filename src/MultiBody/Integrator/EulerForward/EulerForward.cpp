#include "MultiBody/Integrator/EulerForward/EulerForward.h"

#include <iostream>

EulerForward::EulerForward()
{};

void EulerForward::solve0(Eigen::MatrixXd& dofTot,
	Eigen::MatrixXd& M, Eigen::VectorXd& f, 
	std::vector<std::unique_ptr<RigidBody>>& body,
	std::vector<std::unique_ptr<Constraint>>& constraint,
	std::vector<std::unique_ptr<External>>& external) {

	for (int j = 0; j < body.size(); j++) {
		int k = j * kDof;
		dofTot.block<kDof, 1>(k, 0) = body[j]->getDof();
		M.block<kDof, kDof>(k, k) = body[j]->getMass();
		f.segment<kDof>(k) = body[j]->getF();
		//this->print();
	}
	for (int j = 0; j < constraint.size(); j++) {
		constraint[j]->updateConstraint(body, M, f, j);
		//std::cout << "G: " << this->constraint_[j].get().getG() << std::endl;
	}
	for (int j = 0; j < external.size(); j++) {
		external[j]->updateExternal(body, f);
		//std::cout << "Ext: " << this->external_[j].get().getExt() << std::endl;
	}

};

void EulerForward::solve(Eigen::MatrixXd& dofTot,
	Eigen::MatrixXd& M, Eigen::VectorXd& f,
	std::vector<std::unique_ptr<RigidBody>>& body,
	std::vector<std::unique_ptr<Constraint>>& constraint,
	std::vector<std::unique_ptr<External>>& external,
	std::unique_ptr<TimeSim>& time) {

	this->solve0(dofTot, M, f, body, constraint, external);

	for (int i = 0; i < time->getNSteps(); i++) {
		time->step();
		dofTot.col(i + 1) = dofTot.col(i) + M.lu().solve(f) * time->getDt();;
		for (int j = 0; j < body.size(); j++) {
			int k = j * kDof;
			body[j]->updateBody(dofTot.col(i + 1), M, f, k);
			//this->print();
		}
		for (int j = 0; j < constraint.size(); j++) {
			constraint[j]->updateConstraint(body, M, f, j);
			//std::cout << "G: " << this->constraint_[j].get().getG() << std::endl;
		}
		for (int j = 0; j < external.size(); j++) {
			external[j]->updateExternal(body, f);
			//std::cout << "Ext: " << this->external_[j].get().getExt() << std::endl;
		}
	}

};

void EulerForward::print(std::vector<std::unique_ptr<RigidBody>>& body,
	std::unique_ptr<TimeSim>& time) const {

	std::cout << "Actual Time Step: " << time->getTimeActual() << "s" << std::endl;
	for (int j = 0; j < body.size(); j++) {
		std::cout << "Dof: \n" << body[j]->getDof() << std::endl;
		Quaternion q(body[j]->getDof().coeff(10), body[j]->getDof().segment<3>(11));
		std::cout << "Quaternion Constraint: " << 1.0 - q.norm() << std::endl;
	}

}