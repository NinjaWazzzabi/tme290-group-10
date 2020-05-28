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

	float relative_bearing_;
	float distance_;
	uint32_t type_;

public:
	ConeLocation(float relative_bearing, float distance, uint32_t type)
	{
		relative_bearing_ = relative_bearing;
		distance_ = distance;
		type_ = type;
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
