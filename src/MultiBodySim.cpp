#include "MultiBodySim.h"

#include <chrono>

int main()
{

	std::cout << "Start Program" << std::endl;

	double mV = 10.0;
	double JV = 1.0;

	MatrixN m0(3, mV);
	MatrixN J0(3, JV);
	MatrixN m1(3, mV * 2.0);
	MatrixN J1(3, JV * 10.0);

	MatrixN xG(3, 1, 0.0);
	xG[0][0] = 1.0;
	MatrixN xGp(3, 1, 0.0);
	xGp[0][0] = -1.0;

	Quaternion qp0(0.0, MatrixN(3, 1, { {0.23}, {0.17}, {sqrt(1 - 0.0 * 0.0 - 0.23 * 0.23 - 0.17 * 0.17)} }));
	Quaternion q0(1.0, MatrixN(3, 1, 0.0));
	Quaternion qp1(0.0, MatrixN(3, 1, { {0.1}, {0.6}, {sqrt(1 - 0.0 * 0.0 - 0.1 * 0.1 - 0.6 * 0.6)} }));
	Quaternion q1(1.0, MatrixN(3, 1, 0.0));

	MatrixN fExt0(3, 1, { {1.0}, {0.0}, {0.0} });
	MatrixN fExt1(3, 1, 0.0);
	MatrixN mExt0(3, 1, 0.0);
	MatrixN mExt1(3, 1, 1.0);

	RigidBody Body0(m0, J0, xG, q0, xGp, qp0, fExt0, mExt0);
	RigidBody Body1(m1, J1, xG, q1, xGp, qp1, fExt1, mExt1);

	EqualityC EqC(7, 22);

	EulerForward I(0.0, 1.0, 0.001, 0.0,
		std::vector<std::reference_wrapper<RigidBody>> {std::ref(Body0), std::ref(Body1)},
		std::vector<std::reference_wrapper<Constraint>> {std::ref(EqC)});

	auto start = std::chrono::high_resolution_clock::now();
	I.solve();
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

	std::cout << "Time taken by function: " << duration.count() / 1000.0 << " seconds" << std::endl;

	I.printToFile();
	//I.print();

	//std::cout << "DOFs: \n" << Body0.getDof() << std::endl;
	//std::cout << "DOFs: \n" << Body1.getDof() << std::endl;
	//Body.updateXG(MatrixN(3, 1, 1.0));
	//Body.updateDof();
	//std::cout << "DOFs: \n" << Body.getDof() << std::endl;

	//std::cout << "Linear System Solver: \n" << (MatrixN(4, 4, { {2.0, 1.0, 3.0, 5.0}, {0.2, 1.0, 0.7, 8.0}, {1.0, 0.0, 1.0, 1.0}, {0.0, 3.0, 2.0, 9.0} })) / MatrixN(4, 1, { {2.0}, {1.0}, {3.0}, {0.0} }) << std::endl;

	//std::cout << "Linear System Solver: \n" << (Body.getMass() + MatrixN(15, 100.0*DBL_EPSILON)) / Body.getF() << std::endl;

	//std::cout << "Global angular velocity: \n" << Body.getWGlobal() << std::endl;

	//std::cout << "Local angular velocity: \n" << Body.getWLocal() << std::endl;

	//std::cout << "Mass Matrix: \n" << Body.getMass() << std::endl;

	//std::cout << "F Vector: \n" << Body.getF() << std::endl;

	//std::cout << "DOFs: \n" << Body.getDof() << std::endl;

	std::cout << "End Program" << std::endl;

	return 0;
}
