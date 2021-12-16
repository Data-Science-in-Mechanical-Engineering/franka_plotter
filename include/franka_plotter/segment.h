#pragma once
#include <Eigen/Dense>
#include <limits>

class Segment
{
private:
    double _length;

protected:
    void _calculate_length(size_t npoints, const double *intermediate_points);

public:
    double length()                                 const;
    virtual Eigen::Vector2d point(double fraction)  const = 0;
    virtual ~Segment()                              = 0;
};