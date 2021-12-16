#include "../include/franka_plotter/arc.h"

Arc::Arc(const std::array<Eigen::Vector2d, 2> &points, Eigen::Vector2d radii, double angle, bool large_arc, bool sweep)
{
	//https://github.com/MadLittleMods/svg-curve-lib/blob/master/src/c%2B%2B/SVGCurveLib.cpp
	radii = radii.array().abs().matrix();
	_angle = fmod(angle, 360.0) * M_PI / 180.0;

	Eigen::Vector2d difference = (points[0] - points[1]) / 2.0;
	Eigen::Vector2d transformed_point(
		cos(_angle)*difference(0) + sin(_angle)*difference(1),
		-sin(_angle)*difference(0) + cos(_angle)*difference(1));

	// Ensure radii are large enough
	double radii_check = pow(transformed_point(0), 2) / pow(radii(0), 2) + pow(transformed_point(1), 2) / pow(radii(1), 2);
	_radii = (radii_check > 1) ? (sqrt(radii_check) * radii) : radii;

	// Step #2: Compute transformed_center
	double square_numerator = pow(_radii(0), 2)*pow(_radii(1), 2) - pow(_radii(0), 2)*pow(transformed_point(1), 2) - pow(_radii(1), 2)*pow(transformed_point(0), 2);
	double square_root_denominator = pow(_radii(0), 2)*pow(transformed_point(1), 2) + pow(_radii(1), 2)*pow(transformed_point(0), 2);
	double radicand = square_numerator / square_root_denominator;
	// Make sure this never drops below zero because of precision
	radicand = (radicand < 0) ? 0 : radicand;
	double coef = ((large_arc != sweep) ? 1 : -1) * sqrt(radicand);
	Eigen::Vector2d transformed_center(
		coef*((_radii(0)*transformed_point(1))/_radii(1)),
		coef*(-(_radii(1)*transformed_point(0))/_radii(0)));

	// Step #3: Compute center
	_center = Eigen::Vector2d(
		cos(_angle)*transformed_center(0) - sin(_angle)*transformed_center(1) + ((points[0](0)+points[1](0))/2),
		sin(_angle)*transformed_center(0) + cos(_angle)*transformed_center(1) + ((points[0](1)+points[1](1))/2));
	
	// Step #4: Compute start/sweep angles
	// Start angle of the elliptical arc prior to the stretch and rotate operations.
	// Difference between the start and end angles
	Eigen::Vector2d start_vector = ((transformed_point - transformed_center).array() / _radii.array()).matrix();
	_start_angle = atan2(start_vector(1), start_vector(0));
	Eigen::Vector2d end_vector = ((-transformed_point - transformed_center).array() / _radii.array()).matrix();
	_sweep_angle = atan2(start_vector(0)*end_vector(1) - start_vector(1)*end_vector(0), start_vector.dot(end_vector));
	
	if (!sweep && _sweep_angle > 0) _sweep_angle -= 2 * M_PI;
	else if(sweep && _sweep_angle < 0) _sweep_angle += 2 * M_PI;
	_sweep_angle = (_sweep_angle > 0.0) ? fmod(_sweep_angle, 2 * M_PI) : -fmod(-_sweep_angle, 2 * M_PI);

	_calculate_length(0, nullptr);
}

Eigen::Vector2d Arc::point(double fraction) const
{	
	double angle = _start_angle + _sweep_angle * fraction;
	return Eigen::Vector2d(
		cos(_angle)*cos(angle)*_radii(0) - sin(_angle)*sin(angle)*_radii(1) + _center(0),
		sin(_angle)*cos(angle)*_radii(0) + cos(_angle)*sin(angle)*_radii(1) + _center(1));
}