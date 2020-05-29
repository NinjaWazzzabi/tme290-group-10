#pragma once

#include <stdint.h>
#include <math.h>

class KiwiLocation
{
	static constexpr double WIDTH = 0.16; // Meters
	double x_;
	double y_;
	double w_;
	double h_;
	double relative_bearing_;
	double distance_;

public:

	KiwiLocation(double x, double y, double w, double h, double relative_bearing, double distance)
	{
		x_ = x;
		y_ = y;
		w_ = w;
		h_ = h;
		relative_bearing_ = relative_bearing;
		distance_ = distance;
	}

	KiwiLocation(double x, double y, double w, double h,uint32_t image_width, double camera_fov, uint32_t kiwi_width_imagespace, uint32_t kiwi_centre_imagespace)
	{
		x_ = x;
		y_ = y;
		w_ = w;
		h_ = h;
		double kiwi_fov = camera_fov * (double(WIDTH) / double(kiwi_width_imagespace));
		// Using hypotenuse theorem
		distance_ = sqrt(kiwi_fov * kiwi_fov / 4.0 - WIDTH * WIDTH / 4.0);
		relative_bearing_ = (double(kiwi_centre_imagespace) - double(image_width) / 2.0) * (camera_fov / double(image_width));
	}
};