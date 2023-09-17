#include "MultiBodySim.h"

int main()
{

	std::cout << "Start Program" << std::endl;

	double mV = 1.0;
	double JV = 0.5;
	double kV = 1.0;

	MatrixN m(3, mV);
	MatrixN J(3, JV);
	MatrixN xG(3, 1, 5.0);
	Quaternion q(3.0, MatrixN(3,1,2.0));

	RigidBody Body(m, J, xG, q, xG, q);

	std::cout << "Mass Matrix: \n" << Body.getMass() << std::endl;

	std::cout << "DOFs: \n" << Body.getDof() << std::endl;

	std::cout << "End Program" << std::endl;

	return 0;
}
