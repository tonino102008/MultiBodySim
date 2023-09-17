#include "MultiBodySim.h"

int main()
{

	std::cout << "Start Program" << std::endl;

	double mV = 1.0;
	double JV = 0.5;
	double kV = 1.0;

	MatrixN m(3, mV);
	MatrixN J(3, JV);
	MatrixN xG(3, 1, 0.0);
	MatrixN xGp(3, 1, 0.0);
	Quaternion q(0.3, MatrixN(3, 1, { {0.5}, {0.7}, {sqrt(1 - 0.3 * 0.3 - 0.5 * 0.5 - 0.7 * 0.7)} }));
	Quaternion qp(0.125, MatrixN(3, 1, { {0.23}, {0.17}, {sqrt(1 - 0.125 * 0.125 - 0.23 * 0.23 - 0.17 * 0.17)} }));

	RigidBody Body(m, J, xG, q, xGp, qp);

	std::cout << "Prova: \n" << q+qp << std::endl;

	std::cout << "Global angular velocity: \n" << Body.getWGlobal() << std::endl;

	std::cout << "Local angular velocity: \n" << Body.getWLocal() << std::endl;

	std::cout << "Mass Matrix: \n" << Body.getMass() << std::endl;

	std::cout << "F Vector: \n" << Body.getF() << std::endl;

	std::cout << "DOFs: \n" << Body.getDof() << std::endl;

	std::cout << "End Program" << std::endl;

	return 0;
}
