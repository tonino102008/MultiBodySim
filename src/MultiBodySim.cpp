#include "MultiBodySim.h"

#include<vector> 

int main()
{

	std::cout << "Start Program" << std::endl;

	int m = 1;
	int k = 1;

	Quaternion q(1, std::vector<double> {1, 1, 1});
	std::cout << "Quaternion q: " << q.getScalar() << " " << q.getVector()[0] << std::endl;

	Integrator I;
	std::cout << "Integrator I: " << I.getDt() << " " << I.getTimeEnd() << std::endl;

	std::cout << "End Program" << std::endl;

	return 0;
}
