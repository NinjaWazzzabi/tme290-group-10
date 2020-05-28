#pragma once

#include <vector>
#include <string>
#include <iostream>

#define CONE_START        0
#define CONE_LEFT         1
#define CONE_RIGHT        2
#define CONE_INTERSECTION 3

class ConeLocation
{
	float x_;
	float y_;
	float w_;
	float h_;
	float relative_bearing_;
	float distance_;
	uint32_t type_;

public:
	ConeLocation(float x, float y, float w, float h, float relative_bearing, float distance, uint32_t type)
	{
		x_ = x;
		y_ = y;
		w_ = w;
		h_ = h;
		relative_bearing_ = relative_bearing;
		distance_ = distance;
		type_ = type;
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

	uint32_t type()
	{
		return type_;
	}
};
