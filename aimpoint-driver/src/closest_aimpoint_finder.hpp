#pragma once

#include <vector>
#include <cmath>
#include "cone_location.hpp"
#include "line.hpp"
#include "aimpoint_finder_base.hpp"

using namespace Eigen;

class ClosestAimpointFinder : public AimpointBase
{

	Vector2d steer_one_cone_type(std::vector<ConeLocation> &cones, uint32_t cone_type)
	{
		ConeLocation closest_found = closest_to_kiwi(cones);
		double closest_counterpart_x = cone_type == CONE_LEFT ? screen_width_ : 0;

		return Vector2d((closest_found.x() + closest_counterpart_x) / 2, closest_found.y());
	}

	Vector2d steer_both_cone_types(std::vector<ConeLocation> &cones_left, std::vector<ConeLocation> &cones_right)
	{
		ConeLocation closest_left = closest_to_kiwi(cones_left);
		ConeLocation closest_right = closest_to_kiwi(cones_right);

		double middle_x = (closest_left.x() + closest_right.x()) / 2;
		double middle_y = (closest_left.y() + closest_right.y()) / 2;
		return Vector2d(middle_x, middle_y);
	}

public:
	ClosestAimpointFinder(uint32_t screen_width) : AimpointBase(screen_width)
	{
	}

};