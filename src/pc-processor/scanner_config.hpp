#ifndef PC_SCANNER_CONFIG_HPP
#define PC_SCANNER_CONFIG_HPP

/* struct LidarData */

// Define if the rotator's angle is included in the struct
#define LIDAR_DATA_HAS_ROTATOR_ANGLE

// Define if the struct will handle flags
#undef LIDAR_DATA_HAS_FLAGS

#if defined(LIDAR_DATA_HAS_FLAGS) && !defined(LIDAR_DATA_HAS_ROTATOR_ANGLE)
#error "LIDAR_DATA_HAS_FLAGS is defined, so LIDAR_DATA_HAS_ROTATOR_ANGLE must be defined."
#endif


namespace config
{
	constexpr auto heightDelta = 4000.0; // Height of lidar - height of rotator base, millimeters
	constexpr auto horizontalDelta = 4000.0; // Horizontal distance from center of lidar to center of rotator base, millimeters
}


#endif // PC_SCANNER_CONFIG_HPP

