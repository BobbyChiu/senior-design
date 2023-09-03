#include "simulated_leveler.hpp"


SimulatedLeveler::SimulatedLeveler(double startAngle, double angleRate)
	: startAngle(startAngle), angleRate(angleRate), startTime(std::chrono::steady_clock::now())
{}


double SimulatedLeveler::pollAngle() const
{
	auto nowTime = std::chrono::steady_clock::now();
	auto seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(nowTime - startTime).count() / 1E9;
	return seconds * angleRate + startAngle;
}
