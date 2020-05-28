#pragma once

#include <vector>
#include <string>
#include <iostream>

#define CONE_START        0
#define CONE_LEFT         1
#define CONE_RIGHT        2
#define CONE_INTERSECTION 3

class ConeMeasurment
{
	float x_;
	float y_;
	float w_;
	float h_;
	float relative_bearing_;
	float distance_;
	uint32_t type_;

public:
	ConeMeasurment(float x, float y, float w, float h, float relative_bearing, float distance, uint32_t type)
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

class ConeCoder
{
public:
	ConeCoder() = delete;

	static std::vector<ConeMeasurment> decode(std::string data)
	{
		std::vector<ConeMeasurment> cones;
		uint32_t nbr_of_cones = data.size() / sizeof(ConeMeasurment);
		ConeMeasurment *raw_cones = (ConeMeasurment *)data.c_str();
		for (size_t i = 0; i < nbr_of_cones; i++)
		{
			cones.push_back(*raw_cones);
			raw_cones++;
		}

		return cones;
	}

	static std::string encode(std::vector<ConeMeasurment> cones)
	{
		std::string cone_str = "";

		for (size_t i = 0; i < cones.size(); i++)
		{
			ConeMeasurment current = cones.at(i);
			char *raw = (char *)&current;
			std::string raw_str(raw, raw + sizeof(ConeMeasurment));
			cone_str = cone_str + raw_str;
		}

		return cone_str;
	}
};
