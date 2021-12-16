#pragma once
#include "segment.h"
#include <array>

class Arc : public Segment
{
private:
	Eigen::Vector2d _center, _radii;
	double _angle, _start_angle, _sweep_angle;

public:
	Arc(const std::array<Eigen::Vector2d, 2> &points, Eigen::Vector2d radii, double angle, bool large_arc, bool sweep);
	virtual Eigen::Vector2d point(double fraction)	const;
};