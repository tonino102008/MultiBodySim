#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODY_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODY_H_

#include "Integrators/Integrator.h"
#include "Quaternions/Quaternion.h"

class RigidBody {

public:

private:

	std::vector<double> x_; // State translation vector

	Quaternion q_; // State rotation quaternion

	std::vector<std::vector<double>> m_;

	std::vector<std::vector<double>> J_;

	std::vector<std::vector<double>> M_;

	std::vector<std::vector<double>> f_;

	Integrator I_;

};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODY_H_