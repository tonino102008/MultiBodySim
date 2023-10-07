#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_INTEGRATOR_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_INTEGRATOR_H_

#include "MultiBody/RigidBody/RigidBody.h"
#include "MultiBody/Constraints/Constraint.h"
#include "MultiBody/External/External.h"
#include "TimeSim.h"

#include <memory>
#include <vector>

class Integrator {

public:

	Integrator();

	virtual void solve(Eigen::MatrixXd& dofTot,
		Eigen::MatrixXd& M, Eigen::VectorXd& f,
		std::vector<std::shared_ptr<RigidBody>> body,
		std::vector<std::shared_ptr<Constraint>> constraint,
		std::vector<std::shared_ptr<External>> external,
		std::unique_ptr<TimeSim>& time) = 0;

	virtual void print(std::vector<std::shared_ptr<RigidBody>> body,
		std::unique_ptr<TimeSim>& time) const = 0;

private:

	virtual void solve0(Eigen::MatrixXd& dofTot,
		Eigen::MatrixXd& M, Eigen::VectorXd& f,
		std::vector<std::shared_ptr<RigidBody>> body,
		std::vector<std::shared_ptr<Constraint>> constraint,
		std::vector<std::shared_ptr<External>> external) = 0;

};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_INTEGRATOR_INTEGRATOR_H_