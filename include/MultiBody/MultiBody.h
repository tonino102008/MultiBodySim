#ifndef MULTIBODYSIM_INCLUDE_MULTIBODY_MULTIBODY_H_
#define MULTIBODYSIM_INCLUDE_MULTIBODY_MULTIBODY_H_

#include "RigidBody/RigidBody.h"
#include "Constraints/Constraint.h"
#include "External/External.h"
#include "Integrator/Integrator.h"
#include "TimeSim.h"

#include <Eigen/Dense>
#include <vector>
#include <ostream>

class MultiBody {

public:

	MultiBody(const double timeStart, const double timeEnd,
		const double dt, const double timeActual,
		const int nBody, const int nConstr, const int nExt);

	Eigen::MatrixXd getdofTimeHistory() const;

	Eigen::MatrixXd getMass() const;

	Eigen::VectorXd getF() const;

	void setTime(const TimeSim& time);

	void setBody(const RigidBody& body, const int i);

	template <typename T> void setConstr(const T& constr, const int i) {
		this->constraint_[i] = std::make_shared<T>(constr);
	};

	template <typename T> void setExt(const T& ext, const int i) {
		this->external_[i] = std::make_shared<T>(ext);
	};

	template <typename T> void setIntegr(const T& integr) {
		this->integrator_ = std::make_unique<T>(integr);
	};

	void solve();

	void printToFile() const;

	friend std::ostream& operator<<(std::ostream& out, const MultiBody& I);

protected:

	Eigen::MatrixXd dofTimeHistory_;

	Eigen::MatrixXd M_;

	Eigen::VectorXd f_;

	std::unique_ptr<TimeSim> time_;

	std::vector<std::shared_ptr<RigidBody>> body_;

	std::vector<std::shared_ptr<Constraint>> constraint_;

	std::vector<std::shared_ptr<External>> external_;

	std::unique_ptr<Integrator> integrator_;

	const int nBody_;

	const int nConstr_;

	const int nExt_ ;
	
};

#endif // MULTIBODYSIM_INCLUDE_MULTIBODY_MULTIBODY_H_
