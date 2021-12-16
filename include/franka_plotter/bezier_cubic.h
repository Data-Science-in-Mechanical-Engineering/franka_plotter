#pragma once
#include "segment.h"
#include <array>

class BezierCubic : public Segment
{
private:
    std::array<Eigen::Vector2d, 4> _points;

public:
    BezierCubic(const std::array<Eigen::Vector2d, 4> &points);
	virtual Eigen::Vector2d point(double fraction)  const;
};