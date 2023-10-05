#include "TimeSim.h"

TimeSim::TimeSim(const double timeStart, const double timeEnd,
	const double dt, const double timeActual) :
	timeStart_(timeStart), timeEnd_(timeEnd),
	dt_(dt), timeActual_(timeActual),
	nSteps_((timeEnd - timeStart) / dt)
{};

double TimeSim::getTimeStart() const {
	return this->timeStart_;
};

double TimeSim::getTimeEnd() const {
	return this->timeEnd_;
};

double TimeSim::getDt() const {
	return this->dt_;
};

double TimeSim::getTimeActual() const {
	return this->timeActual_;
};

int TimeSim::getNSteps() const {
	return this->nSteps_;
};

void TimeSim::step() {
	this->timeActual_ += this->dt_;
};

std::ostream& operator<<(std::ostream& out, const TimeSim& t) {
	out << "Actual Time Step: " << t.timeActual_ << "s" << std::endl;
	out << "Simulation starts from: " << t.timeStart_ << "s"
		<< "and finishes at: " << t.timeEnd_ << "s" << std::endl;
	out << "Actual Delta Time: " << t.dt_ << "s" << std::endl;
	return out;
};