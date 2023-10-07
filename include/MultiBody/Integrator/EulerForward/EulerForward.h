#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_EULERFORWARD_EULERFORWARD_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_EULERFORWARD_EULERFORWARD_H_

#include "MultiBody/Integrator/Integrator.h"

#include <Eigen/Dense>

class EulerForward : public Integrator {

public:

	EulerForward();

	void solve(Eigen::MatrixXd& dofTot,
		Eigen::MatrixXd& M, Eigen::VectorXd& f, 
		std::vector<std::shared_ptr<RigidBody>> body,
		std::vector<std::shared_ptr<Constraint>> constraint,
		std::vector<std::shared_ptr<External>> external,
		std::unique_ptr<TimeSim>& time);

	void print(std::vector<std::shared_ptr<RigidBody>> body,
		std::unique_ptr<TimeSim>& time) const;

private:

	void solve0(Eigen::MatrixXd& dofTot,
		Eigen::MatrixXd& M, Eigen::VectorXd& f,
		std::vector<std::shared_ptr<RigidBody>> body,
		std::vector<std::shared_ptr<Constraint>> constraint,
		std::vector<std::shared_ptr<External>> external);

};

#endif //MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_EULERFORWARD_EULERFORWARD_H_