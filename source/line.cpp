#include "../include/franka_plotter/line.h"

Line::Line(const std::array<Eigen::Vector2d, 2> &points)
{
	_points = points;
	_calculate_length(0, nullptr);
}

Eigen::Vector2d Line::point(double fraction) const
{
	if (_points[0] == _points[1]) return _points[0];
	else return _points[0] * (1.0 - fraction) + _points[1] * fraction;
}