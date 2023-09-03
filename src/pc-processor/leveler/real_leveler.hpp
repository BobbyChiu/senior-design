#ifndef LEVELER_REAL_LEVELER_HPP
#define LEVELER_REAL_LEVELER_HPP

#include "leveler.hpp"


class RealLeveler : public Leveler
{
public:
	RealLeveler();

	virtual double pollAngle() const override = 0; // TODO: Implement
};

#endif // LEVELER_REAL_LEVELER_HPP
