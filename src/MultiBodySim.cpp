#include "MultiBodySim.h"

int main()
{

	std::cout << "Start Program" << std::endl;

	int m = 1;
	int k = 1;

	Quaternion q(1, std::vector<double> {1, 1, 1});
	std::cout << "Quaternion q: " << q << std::endl;

	Integrator I;
	std::cout << "Integrator I: \n" << I << std::endl;

	std::cout << "End Program" << std::endl;

	return 0;
}
