#pragma once

#include <stdint.h>
#include <math.h>

class KiwiLocation
{
	static constexpr double WIDTH = 0.16; // Meters

	double relative_bearing_;
	double distance_;

public:

	KiwiLocation(double relative_bearing, double distance)
	{
		relative_bearing_ = relative_bearing;
		distance_ = distance;
	}

	KiwiLocation(uint32_t image_width, double camera_fov, uint32_t kiwi_width_imagespace, uint32_t kiwi_centre_imagespace)
	{
		double kiwi_fov = camera_fov * (double(WIDTH) / double(kiwi_width_imagespace));
		// Using hypotenuse theorem
		distance_ = sqrt(kiwi_fov * kiwi_fov / 4.0 - WIDTH * WIDTH / 4.0);
		relative_bearing_ = (double(kiwi_centre_imagespace) - double(image_width) / 2.0) * (camera_fov / double(image_width));
	}
};