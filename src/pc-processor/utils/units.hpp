#ifndef UTILS_UNITS_HPP
#define UTILS_UNITS_HPP

#include <pcl/pcl_macros.h>


// Length (meters)
constexpr long double operator""_m(long double meters)
{
	return meters;
}


constexpr long double operator""_mm(long double millimeters)
{
	return millimeters / 1000.0;
}


// Angle (radians)
constexpr long double operator""_deg(long double degrees)
{
	return DEG2RAD(degrees);
}


constexpr long double operator""_rad(long double radians)
{
	return radians;
}


#endif // UTILS_UNITS_HPP

