#pragma once

#include <vector>
#include <cmath>
#include "cone_location.hpp"
#include "line.hpp"

using namespace Eigen;

class ClosestAimpointFinder
{

	uint32_t screen_width_;
	double no_cone_steering_strength_;

	double calculate_distance(ConeLocation coneA, ConeLocation coneB)
	{
		double distance;
		distance = sqrt((coneB.x() - coneA.x()) * (coneB.x() - coneA.x()) + (coneB.y() - coneA.y()) * (coneB.y() - coneA.y()));
		return distance;
	}

	uint32_t estimate_cone_side(ConeLocation cone, Vector2d differentiator)
	{
		return cone.x() - differentiator.x() > 0 ? CONE_RIGHT : CONE_LEFT;
	}

	ConeLocation closest_to_kiwi(std::vector<ConeLocation> &cones)
	{
		ConeLocation closest = cones.at(0);
		for (size_t i = 1; i < cones.size(); i++)
		{
			ConeLocation cone_tmp = cones.at(i);
			if (cone_tmp.y() > closest.y())
			{
				closest = cone_tmp;
			}
		}
		return closest;
	}

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

	Vector2d find_aimpoint(std::vector<ConeLocation> &cones_left, std::vector<ConeLocation> &cones_right, Vector2d prev_aimpoint)
	{
		Vector2d aimpoint;
		if (cones_left.size() < 1 && cones_right.size() < 1)
		{
			// No cones found
			aimpoint = prev_aimpoint;
		}
		else if (cones_left.size() < 1)
		{
			// Cones on the right
			aimpoint = steer_one_cone_type(cones_right, CONE_RIGHT);
		}
		else if (cones_right.size() < 1)
		{
			// Cones on the left
			aimpoint = steer_one_cone_type(cones_left, CONE_LEFT);
		}
		else
		{
			// Cones on both sides
			aimpoint = steer_both_cone_types(cones_left, cones_right);
		}

		return aimpoint;
	}

public:
	ClosestAimpointFinder(uint32_t screen_width, double no_cone_steering_strength = 400.0)
	{
		screen_width_ = screen_width;
		no_cone_steering_strength_ = no_cone_steering_strength;
	}

	Vector2d find_aimpoint(std::vector<ConeLocation> &cones, Vector2d prev_aimpoint)
	{
		// Group cones into cones_left and cones_right
		std::vector<ConeLocation> cones_left;
		std::vector<ConeLocation> cones_right;
		std::vector<ConeLocation> cones_intersection;
		for (ConeLocation cone_location : cones)
		{
			if (cone_location.type() == CONE_LEFT)
			{
				cones_left.push_back(cone_location);
			}
			else if (cone_location.type() == CONE_RIGHT)
			{
				cones_right.push_back(cone_location);
			}
			else if (cone_location.type() == CONE_INTERSECTION)
			{
				cones_intersection.push_back(cone_location);
			}
		}

		// If the majority cones are intersection cones,
		// add them to the list through the cone side estimator
		if (cones_intersection.size() > cones_left.size() + cones_right.size())
		{
			for (ConeLocation cone_location : cones_intersection)
			{
				if (estimate_cone_side(cone_location, prev_aimpoint) == CONE_LEFT)
				{
					cones_left.push_back(cone_location);
				}
				else
				{
					cones_right.push_back(cone_location);
				}
			}
		}

		return find_aimpoint(cones_left, cones_right, prev_aimpoint);
	}
};