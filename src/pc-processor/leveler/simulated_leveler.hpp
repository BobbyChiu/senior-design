#ifndef LEVELER_SIMULATED_LEVELER_HPP
#define LEVELER_SIMULATED_LEVELER_HPP

#include <chrono>

#include "leveler.hpp"


class SimulatedLeveler : public Leveler
{
public:
	SimulatedLeveler(double startAngle, double angleRate);

	double pollAngle() const override;

private:
	const double startAngle;
	const double angleRate;

	const std::chrono::time_point<std::chrono::steady_clock> startTime;

};

#endif // LEVELER_SIMULATED_LEVELER_HPP