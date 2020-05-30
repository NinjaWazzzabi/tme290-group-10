#pragma once

#include <vector>
#include <cmath>
#include "cone_location.hpp"
#include "line.hpp"

using namespace Eigen;

class AimpointFinder
{

	double calculate_distance(ConeLocation coneA, ConeLocation coneB)
	{
		double distance;
		distance = sqrt((coneB.x() - coneA.x()) * (coneB.x() - coneA.x()) + (coneB.y() - coneA.y()) * (coneB.y() - coneA.y()));
		return distance;
	}

	Vector2d steer_one_cone_type(std::vector<ConeLocation> &cones, uint32_t cone_type)
	{
		double avg_height_pos = 0.0;

		for (auto cone : cones)
		{
			avg_height_pos += cone.y();
		}

		avg_height_pos /= double(cones.size());

		// Uses known image sizes. Should be switched to bearing and distance
		if (cone_type == CONE_LEFT)
		{
			// Vehicle staring towards the left side of track -> turn right
			return Vector2d(400 + 1280 / 2, avg_height_pos);
		}
		else
		{
			// Vehicle staring towards the right side of track -> turn left
			return Vector2d(-400 + 1280 / 2, avg_height_pos);
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
		// Uses known image sizes. Should be switched to bearing and distance
		return Vector2d((avg_left_pos + avg_right_pos) / 2.0, avg_height_pos);

		// NEXT IDEA:

		// lines_between_cones{}
		// For all possible permutations:
			// Connect lines between between the cones
			// IF there are NO LINES CROSSING
				// add "line permutation" to lines_between_cones

		// // The lines should now build a ladder

		// Calculate aimpoint
	}

public:
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
			else if  (cone_location.type() == CONE_INTERSECTION)
			{
				cones_intersection.push_back(cone_location);
			}
		}


		Vector2d aimpoint;
		if (cones_intersection.size() > cones_left.size() + cones_right.size())
		{
			// In intersection, slowly move aimpoint towards middle of screen
			aimpoint = {(1280.0f/2) * 0.05 + 0.95 * prev_aimpoint.x(),(720.0f/2) *0.05 + 0.95 * prev_aimpoint.y()};
		}
		else if (cones_left.size() < 1 && cones_right.size() < 1)
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
};