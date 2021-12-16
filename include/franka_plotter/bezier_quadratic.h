#pragma once
#include "segment.h"
#include <array>

class BezierQuadratic : public Segment
{
private:
    std::array<Eigen::Vector2d, 3> _points;

public:
    BezierQuadratic(const std::array<Eigen::Vector2d, 3> &points);
	virtual Eigen::Vector2d point(double fraction)  const;
};