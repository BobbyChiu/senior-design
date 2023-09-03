#ifndef LEVELER_LEVELER_HPP
#define LEVELER_LEVELER_HPP


class Leveler
{
public:
	// Return current rotational angle of the leveler in radians
	virtual double pollAngle() const = 0;
};

#endif // LEVELER_LEVELER_HPP
