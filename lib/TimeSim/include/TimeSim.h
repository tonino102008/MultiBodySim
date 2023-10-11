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

	/**
	 * @brief 
	 * @param timeStart 
	 * @param timeEnd 
	 * @param dt 
	 * @param timeActual 
	*/
	TimeSim(const double timeStart, const double timeEnd,
		const double dt, const double timeActual);

	/**
	 * @brief 
	 * @return 
	*/
	double getTimeStart() const;

	/**
	 * @brief 
	 * @return 
	*/
	double getTimeEnd() const;

	/**
	 * @brief 
	 * @return 
	*/
	double getDt() const;

	/**
	 * @brief 
	 * @return 
	*/
	double getTimeActual() const;

	/**
	 * @brief 
	 * @return 
	*/
	int getNSteps() const;

	/**
	 * @brief 
	*/
	void step();

	/**
	 * @brief 
	 * @param out 
	 * @param t 
	 * @return 
	*/
	friend std::ostream& operator<<(std::ostream& out, const TimeSim& t);

private:

	const double timeStart_;

	const double timeEnd_;

	const double dt_;

	double timeActual_;

	const int nSteps_;

};

#endif // MULTIBODYSIM_LIB_TIMESIM_TIMESIM_H_