#pragma once

#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace Eigen;
class Line
{

	Hyperplane<float, 2> path_hyper_line_;
	Vector2f start_;
	Vector2f end_;
	float length_;

public:
	Line(Vector2f start, Vector2f end)
	{
		this->start_ = start;
		this->end_ = end;
		this->length_ = (end - start).norm();
		this->path_hyper_line_ = Hyperplane<float, 2>::Through(start, end);
	}

	Vector2f start()
	{
		return start_;
	}
	Vector2f end()
	{
		return end_;
	}
	float length()
	{
		return length_;
	}

	bool contains_point(Vector2f point)
	{
		float dist_a = (point - this->start()).norm();
		float dist_b = (this->end() - point).norm();
		return dist_a + dist_b <= this->length() + 0.001f;
	}

	bool intersects_line(Line line)
	{
		// Check if lines are parallel and overlapping
		if (contains_point(line.start()) ||
			contains_point(line.end()) ||
			line.contains_point(start()) ||
			line.contains_point(end()))
		{
			return true;
		}

		// Check if intersection point is within both lines
		Vector2f intersection_point = this->path_hyper_line_.intersection(line.path_hyper_line_);
		if (contains_point(intersection_point) && line.contains_point(intersection_point))
		{
			return true;
		}

		return false;
	}
};