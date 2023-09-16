#include "MultiBodySim.h"

int main()
{

	std::cout << "Start Program" << std::endl;

	int m = 1;
	int k = 1;

	VectorN v(3);
	v[1] = 1.0;
	Quaternion q(1.0, v);
	std::cout << "Quaternion q: " << q << std::endl;

	Integrator I;
	std::cout << "Integrator I: \n" << I << std::endl;

	std::cout << "End Program" << std::endl;

	return 0;
}
