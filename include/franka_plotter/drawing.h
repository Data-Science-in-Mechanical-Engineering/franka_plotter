#pragma once
#include "segment.h"
#include <string>
#include <vector>

class Drawing
{
private:
    double _width;
    double _height;
	std::vector<Segment*> _segments;
	
public:
    Drawing(std::string filename);
	size_t segment_number() const;
	const Segment &segment(size_t number) const;
    double width() const;
	double height() const;
    ~Drawing();
};