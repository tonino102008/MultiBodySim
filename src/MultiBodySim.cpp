#include "MultiBodySim.h"

#include <chrono>
#include <Eigen/Dense>
#include <iostream>

int main()
{

	std::cout << "Start Program" << std::endl;

	// DATA

	double mV = 10.0;
	double JV = 1.0;

	Eigen::Matrix3d m0 = Eigen::Matrix3d::Identity() * mV;
	Eigen::Matrix3d J0 = Eigen::Matrix3d::Identity() * JV;
	Eigen::Matrix3d m1 = Eigen::Matrix3d::Identity() * mV * 2.0;
	Eigen::Matrix3d J1 = Eigen::Matrix3d::Identity() * JV * 10.0;

	//Eigen::Vector3d xG0(1.0, 0.0, 0.0);
	//Eigen::Vector3d xGp0(-1.0, 0.0, 0.0);

	Eigen::Vector3d xG0(0.0, 0.0, 0.0);
	Eigen::Vector3d xGp0(0.0, 0.0, 0.0);

	Eigen::Vector3d xG1(1.0, 1.0, 0.0);
	Eigen::Vector3d xGp1(-1.0, 0.0, 0.0);

	//Quaternion qp0(0.0, Eigen::Vector3d (0.23, 0.17, sqrt(1.0 - 0.0 * 0.0 - 0.23 * 0.23 - 0.17 * 0.17)));
	Quaternion qp0(0.0, Eigen::Vector3d::Zero());
	Quaternion q0(1.0, Eigen::Vector3d::Zero());
	//Quaternion qp1(0.0, Eigen::Vector3d(0.1, 0.6, sqrt(1.0 - 0.0 * 0.0 - 0.1 * 0.1 - 0.6 * 0.6)));
	Quaternion qp1(0.0, Eigen::Vector3d::Zero());
	Quaternion q1(1.0, Eigen::Vector3d::Zero());

	const Eigen::Vector3d p0(Eigen::Vector3d::Zero());
	const Eigen::Vector3d p1(1.0, 0.1, 0.0);
	const Eigen::Vector3d axisY(0.0, 1.0, 0.0);
	const Eigen::Vector3d axisZ(0.0, 0.0, 1.0);

	// END - DATA
	// 
	// MULTIBODY

	//MultiBody MB(0.0, 10.0, 0.001, 0.0, 2, 1, 3);

	//MB.setBody(RigidBody(m0, J0, xG0, q0, xGp0, qp0), 0);
	//MB.setBody(RigidBody(m1, J1, xG1, q1, xGp1, qp1), 1);

	//MB.setConstr(EqualityC(7, 22), 0);

	//MB.setExt(Spring(0, 1, 10.0, 0.0, p0, p0, axisY), 0);
	//MB.setExt(Spring0(1, 100.0, 0.0, p0, axisZ), 1);
	//MB.setExt(Spring(0, 1, 10.0, 0.0, p0, p1, axisZ), 2);

	MultiBody MB(0.0, 10.0, 0.001, 0.0, 5, 0, 1);

	MB.setBody(RigidBody(m0, J0, xG0, q0, xGp0, qp0), 0);
	MB.setBody(RigidBody(m0, J0, xG0, q0, xGp0, qp0), 1);
	MB.setBody(RigidBody(m0, J0, xG0, q0, xGp0, qp0), 2);
	MB.setBody(RigidBody(m0, J0, xG0, q0, xGp0, qp0), 3);
	MB.setBody(RigidBody(m0, J0, xG0, q0, xGp0, qp0), 4);

	//MB.setConstr(EqualityC(7, 22), 0);

	MB.setExt(Spring0(0, 10.0, 0.0, p0, axisZ), 0);
	MB.setExt(Spring0(1, 10.0, 0.0, p0, axisZ), 0);
	MB.setExt(Spring0(2, 10.0, 0.0, p0, axisZ), 0);
	MB.setExt(Spring0(3, 10.0, 0.0, p0, axisZ), 0);
	MB.setExt(Spring(0, 4, 10.0, 0.0, p1, p0, axisZ), 0);
	MB.setExt(Spring(1, 4, 10.0, 0.0, p1, p0, axisZ), 0);
	MB.setExt(Spring(2, 4, 10.0, 0.0, p1, p0, axisZ), 0);
	MB.setExt(Spring(3, 4, 10.0, 0.0, p1, p0, axisZ), 0);

	MB.setIntegr(EulerForward());

	// END - MULTIBODY
	//
	// SOLVE EOMs

	auto start = std::chrono::high_resolution_clock::now();
	MB.solve();
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

	std::cout << "Time taken by function: " << duration.count() / 1000.0 << " seconds" << std::endl;

	MB.printToFile();

	// END - SOLVE EOMs

	std::cout << "End Program" << std::endl;

	return 0;
}
