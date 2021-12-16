#include "../include/franka_plotter/bezier_quadratic.h"

BezierQuadratic::BezierQuadratic(const std::array<Eigen::Vector2d, 3> &points)
{
	_points = points;
	double intermediate_points[] = { (points[0] - points[1]).norm() / ((points[0] - points[1]).norm() + (points[1] - points[2]).norm()) };
	_calculate_length(1, intermediate_points);
}

Eigen::Vector2d BezierQuadratic::point(double fraction) const
{
	return pow(1.0 - fraction, 2)*_points[0] + 2*(1.0 - fraction)*fraction*_points[1] + pow(fraction, 2)*_points[2];
}