#pragma once
#include "segment.h"
#include <array>

class Line : public Segment
{
private:
	std::array<Eigen::Vector2d, 2> _points;

public:
    Line(const std::array<Eigen::Vector2d, 2> &points);
	virtual Eigen::Vector2d point(double fraction)	const;
};