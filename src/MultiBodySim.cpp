#include "MultiBodySim.h"

#include <chrono>
#include <Eigen/Dense>
#include <iostream>

int main()
{

	std::cout << "Start Program" << std::endl;

	// DATA

	double mV = 2.5;
	double JV = 10.0;

	Eigen::Matrix3d m0 = Eigen::Matrix3d::Identity() * mV * 10.0;
	Eigen::Matrix3d J0 = Eigen::Matrix3d::Identity() * JV * 10.0;
	Eigen::Matrix3d m1 = Eigen::Matrix3d::Identity() * mV;
	Eigen::Matrix3d J1 = Eigen::Matrix3d::Identity() * JV;

	Eigen::Vector3d xG0(0.0, 0.0, 0.0);
	Eigen::Vector3d xGp0(0.0, 0.0, 0.0);

	Eigen::Vector3d xG1(1.0, 1.0, 0.0);
	Eigen::Vector3d xGp1(0.0, 0.0, 0.0);

	Eigen::Vector3d xG2(1.0, -1.0, 0.0);
	Eigen::Vector3d xGp2(0.0, 0.0, 0.0);

	Eigen::Vector3d xG3(-2.0, 1.0, 0.0);
	Eigen::Vector3d xGp3(0.0, 0.0, 0.0);

	Eigen::Vector3d xG4(-2.0, -1.0, 0.0);
	Eigen::Vector3d xGp4(0.0, 0.0, 0.0);

	Quaternion qp0(0.0, Eigen::Vector3d::Zero());
	Quaternion q0(1.0, Eigen::Vector3d::Zero());
	
	Quaternion qp1(0.0, Eigen::Vector3d::Zero());
	Quaternion q1(1.0, Eigen::Vector3d::Zero());

	const Eigen::Vector3d p0(Eigen::Vector3d::Zero());
	const Eigen::Vector3d p1(1.0, 1.0, 0.0);
	const Eigen::Vector3d p2(1.0, -1.0, 0.0);
	const Eigen::Vector3d p3(-2.0, 1.0, 0.0);
	const Eigen::Vector3d p4(-2.0, -1.0, 0.0);

	const Eigen::Vector3d axisX(1.0, 0.0, 0.0);
	const Eigen::Vector3d axisY(0.0, 1.0, 0.0);
	const Eigen::Vector3d axisZ(0.0, 0.0, 1.0);

	// END - DATA
	// 
	// MULTIBODY

	MultiBody MB(0.0, 30.0, 0.001, 0.0, 5, 8, 12);

	MB.setBody(RigidBody(m0, J0, xG0, q0, xGp0, qp0), 0);
	MB.setBody(RigidBody(m1, J1, xG1, q1, xGp1, qp1), 1);
	MB.setBody(RigidBody(m1, J1, xG2, q1, xGp2, qp1), 2);
	MB.setBody(RigidBody(m1, J1, xG3, q1, xGp3, qp1), 3);
	MB.setBody(RigidBody(m1, J1, xG4, q1, xGp4, qp1), 4);

	MB.setConstr(EqualityC(0, 1, p1, p0, axisX), 0);
	MB.setConstr(EqualityC(0, 2, p2, p0, axisX), 1);
	MB.setConstr(EqualityC(0, 3, p3, p0, axisX), 2);
	MB.setConstr(EqualityC(0, 4, p4, p0, axisX), 3);
	MB.setConstr(EqualityC(0, 1, p1, p0, axisY), 4);
	MB.setConstr(EqualityC(0, 2, p2, p0, axisY), 5);
	MB.setConstr(EqualityC(0, 3, p3, p0, axisY), 6);
	MB.setConstr(EqualityC(0, 4, p4, p0, axisY), 7);

	MB.setExt(Spring0(1, 1000.0, 0.0, p0, p0, axisZ), 0);
	MB.setExt(Spring0(2, 1000.0, 0.0, p0, p0, axisZ), 1);
	MB.setExt(Spring0(3, 1000.0, 0.0, p0, p0, axisZ), 2);
	MB.setExt(Spring0(4, 1000.0, 0.0, p0, p0, axisZ), 3);
	MB.setExt(Spring(0, 1, 100.0, 0.0, p1, p0, axisZ), 4);
	MB.setExt(Spring(0, 2, 100.0, 0.0, p2, p0, axisZ), 5);
	MB.setExt(Spring(0, 3, 100.0, 0.0, p3, p0, axisZ), 6);
	MB.setExt(Spring(0, 4, 100.0, 0.0, p4, p0, axisZ), 7);

	MB.setExt(Damper(0, 1, 10.0, p1, p0, axisZ), 8);
	MB.setExt(Damper(0, 2, 10.0, p2, p0, axisZ), 9);
	MB.setExt(Damper(0, 3, 10.0, p3, p0, axisZ), 10);
	MB.setExt(Damper(0, 4, 10.0, p4, p0, axisZ), 11);

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
