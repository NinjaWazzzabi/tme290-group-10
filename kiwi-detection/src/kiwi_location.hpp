#pragma once

#include <stdint.h>
#include <math.h>
#include <stdio.h>

class KiwiLocation
{
	static constexpr double KIWI_WIDTH = 0.16; // Meters
	static constexpr double KIWI_HEIGHT = 0.11; // Meters
	static constexpr double CAMERA_HEIGHT = 0.091; // Meters
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

	KiwiLocation(double x, double y, double w, double h, uint32_t image_width, uint32_t image_height, double camera_fov_x, double camera_fov_y, uint32_t, uint32_t kiwi_centre_imagespace)
	{
		x_ = x;
		y_ = y;
		w_ = w;
		h_ = h;
		// Using hypotenuse theorem
		// double kiwi_fov = camera_fov * (double(kiwi_width_imagespace) / double(image_width));
		// distance_ = WIDTH / (2 * tan(kiwi_fov * M_PI / (2 * 180)));

		double angle_to_kiwi_top = camera_fov_y / double(image_height) * (image_height / 2 - y);
		distance_ = (KIWI_HEIGHT - CAMERA_HEIGHT) / tan(angle_to_kiwi_top * M_PI / 180.0);
		relative_bearing_ = (double(kiwi_centre_imagespace) - double(image_width) / 2.0) * (camera_fov_x / double(image_width));
	}

	float x()
	{
		return x_;
	}

	float y()
	{
		return y_;
	}

	float w()
	{
		return w_;
	}
	
	float h()
	{
		return h_;
	}

	float distance()
	{
		return distance_;
	}

	float relative_bearing()
	{
		return relative_bearing_;
	}
};