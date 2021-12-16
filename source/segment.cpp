#include "../include/franka_plotter/segment.h"
#include <list>

void Segment::_calculate_length(size_t npoints, const double *intermediate_points)
{
	std::list<std::pair<double, Eigen::Vector2d>> list;
	list.push_back(std::pair<double, Eigen::Vector2d>(0.0, point(0.0)));
	for (size_t i = 0; i < npoints; i++) list.push_back(std::pair<double, Eigen::Vector2d>(intermediate_points[i], point(intermediate_points[i])));
	list.push_back(std::pair<double, Eigen::Vector2d>(1.0, point(1.0)));
	double old_length = std::numeric_limits<double>::quiet_NaN();
	while (true)
	{
		_length = 0.0;
		for (auto i = list.begin(); std::next(i) != list.end(); std::advance(i, 1))
		{
			_length += (i->second - std::next(i)->second).norm();
		}
		if (old_length == old_length && abs(_length - old_length) <= 0.0001*_length) break;
		for (auto i = list.begin(); std::next(i) != list.end(); std::advance(i, 2))
		{
			double fraction = (i->first + std::next(i)->first) / 2;
			list.insert(std::next(i), std::pair<double, Eigen::Vector2d>(fraction, point(fraction)));
		}
		old_length = _length;
	}
}

double Segment::length() const
{
	return _length;
}

Segment::~Segment()
{}