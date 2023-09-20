#include "MultiBodySim.h"

#include <chrono>

int main()
{

	std::cout << "Start Program" << std::endl;

	double mV = 1.0;
	double JV = 0.5;

	MatrixN m(3, mV);
	MatrixN J(3, JV);
	MatrixN xG(3, 1, 0.0);
	//xG[0][0] = 1.0;
	MatrixN xGp(3, 1, 0.0);
	//xGp[0][0] = -1.0;
	//Quaternion q(0.3, MatrixN(3, 1, { {0.5}, {0.7}, {sqrt(1 - 0.3 * 0.3 - 0.5 * 0.5 - 0.7 * 0.7)} }));
	Quaternion qp(0.0, MatrixN(3, 1, { {0.23}, {0.17}, {sqrt(1 - 0.0 * 0.0 - 0.23 * 0.23 - 0.17 * 0.17)} }));
	Quaternion q(1.0, MatrixN(3, 1, 0.0));
	//Quaternion qp(0.0, MatrixN(3, 1, 0.0));

	RigidBody Body(m, J, xG, q, xGp, qp);

	EulerForward I(0.0, 1.0, 0.001, 0.0, std::vector<std::reference_wrapper<RigidBody>> {std::ref(Body)});

	auto start = std::chrono::high_resolution_clock::now();
	I.solve();
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

	std::cout << "Time taken by function: " << duration.count() / 1000.0 << " seconds" << std::endl;

	I.printToFile();

	//std::cout << "DOFs: \n" << Body.getDof() << std::endl;
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
