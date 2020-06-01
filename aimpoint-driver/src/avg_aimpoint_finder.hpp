#pragma once

#include <vector>
#include <cmath>
#include "cone_location.hpp"
#include "line.hpp"
#include "aimpoint_finder_base.hpp"

using namespace Eigen;

class AvgAimpointFinder : public AimpointBase
{
	double no_cone_steering_strength_;

	Vector2d steer_one_cone_type(std::vector<ConeLocation> &cones, uint32_t cone_type)
	{
		double avg_height_pos = 0.0;

		for (auto cone : cones)
		{
			avg_height_pos += cone.y();
		}

		avg_height_pos /= double(cones.size());

		if (cone_type == CONE_LEFT)
		{
			// Vehicle staring towards the left side of track -> turn right
			return Vector2d(no_cone_steering_strength_ + screen_width_ / 2, avg_height_pos);
		}
		else
		{
			// Vehicle staring towards the right side of track -> turn left
			return Vector2d(-no_cone_steering_strength_ + screen_width_ / 2, avg_height_pos);
		}
	}

	Vector2d steer_both_cone_types(std::vector<ConeLocation> &cones_left, std::vector<ConeLocation> &cones_right)
	{
		double avg_height_pos = 0.0;

		double avg_left_pos = 0.0;
		for (auto cone : cones_left)
		{
			avg_left_pos += cone.x();
			avg_height_pos += cone.y();
		}
		avg_left_pos /= double(cones_left.size());

		double avg_right_pos = 0.0;
		for (auto cone : cones_right)
		{
			avg_right_pos += cone.x();
			avg_height_pos += cone.y();
		}
		avg_right_pos /= double(cones_right.size());

		avg_height_pos /= double(cones_left.size() + cones_right.size());
		return Vector2d((avg_left_pos + avg_right_pos) / 2.0, avg_height_pos);
	}

public:
	AvgAimpointFinder(uint32_t screen_width, double no_cone_steering_strength = 400.0)
		: AimpointBase(screen_width)
	{
		no_cone_steering_strength_ = no_cone_steering_strength;
	}
};