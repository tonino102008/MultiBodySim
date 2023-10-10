/*****************************************************************//**
 * \file   TimeSim.h
 * \brief  
 * 
 * \author Antonio Cioffi
 * \date   October 2023
 *********************************************************************/

#ifndef MULTIBODYSIM_LIB_TIMESIM_TIMESIM_H_
#define MULTIBODYSIM_LIB_TIMESIM_TIMESIM_H_

#include <cmath>
#include <ostream>

class TimeSim {

public:

	TimeSim(const double timeStart, const double timeEnd,
		const double dt, const double timeActual);

	double getTimeStart() const;

	double getTimeEnd() const;

	double getDt() const;

	double getTimeActual() const;

	int getNSteps() const;

	void step();

	friend std::ostream& operator<<(std::ostream& out, const TimeSim& t);

private:

	const double timeStart_;

	const double timeEnd_;

	const double dt_;

	double timeActual_;

	const int nSteps_;

};

#endif // MULTIBODYSIM_LIB_TIMESIM_TIMESIM_H_