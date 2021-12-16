#include "../include/franka_plotter/bezier_cubic.h"

BezierCubic::BezierCubic(const std::array<Eigen::Vector2d, 4> &points)
{
	_points = points;
	double length_estimate = (points[0] - points[1]).norm() + (points[1] - points[2]).norm() + (points[2] - points[3]).norm();
	double intermediate_points[] = { (points[0] - points[1]).norm() / length_estimate, ((points[0] - points[1]).norm() + (points[1] - points[2]).norm()) / length_estimate };
	_calculate_length(2, intermediate_points);
}

Eigen::Vector2d BezierCubic::point(double fraction) const
{
	return pow(1.0 - fraction, 3)*_points[0] + 3*pow(1.0 - fraction, 2)*fraction*_points[1] + 3*(1.0 - fraction)*pow(fraction, 2)*_points[2] + pow(fraction, 3)*_points[3];
}