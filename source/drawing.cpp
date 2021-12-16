#include "../include/franka_plotter/drawing.h"
#include "../include/franka_plotter/arc.h"
#include "../include/franka_plotter/bezier_cubic.h"
#include "../include/franka_plotter/bezier_quadratic.h"
#include "../include/franka_plotter/line.h"
#include <fstream>

Drawing::Drawing(std::string filename)
{
    //Read file
	std::string svg;
	{
		std::ifstream stream(filename, std::ios::binary);
		if (!stream.good()) throw std::runtime_error("Drawing::Drawing: could not open file");
		stream.seekg(0, std::ios::end);
		svg.resize(stream.tellg());
		stream.seekg(0, std::ios::beg);
		stream.read(&svg[0], svg.size());
		if (!stream.good() || svg.empty()) throw std::runtime_error("Drawing::Drawing: could not read file");
	}

    //Find width
    if (strstr(svg.c_str(), "width=\"") == nullptr) throw std::runtime_error("Drawing::Drawing: width not found");
    _width = strtod(strstr(svg.c_str(), "width=\"") + strlen("width=\""), nullptr);
    if (_width == 0.0) throw std::runtime_error("Drawing::Drawing: width invalid");

    //Find height
    if (strstr(svg.c_str(), "height=\"") == nullptr) throw std::runtime_error("Drawing::Drawing: height not found");
    _height = strtod(strstr(svg.c_str(), "height=\"") + strlen("height=\""), nullptr);
    if (_height == 0.0) throw std::runtime_error("Drawing::Drawing: height invalid");

    //Find all paths
    Eigen::Vector2d current = Eigen::Vector2d::Zero();
    const char *path = strstr(svg.c_str(), "d=\"");
    while (path != nullptr)
    {
        path += strlen("d=\"");
        Eigen::Vector2d path_start = Eigen::Vector2d::Zero();
		bool path_started = false;
        char previous_typ = '\0';
        Eigen::Vector2d previous_bezier_reflection;
		while (previous_typ != '\"')
        {
            while (*path == ' ') path++;
            char typ = *path;
            switch (typ)
            {
                case 'M':
                case 'm':
                {
                    //Move to
                    Eigen::Vector2d value;
                    for (size_t i = 0; i < 2; i++)
                    {
                        while (*path < '0' || *path > '9') path++;
                        char *end;
                        value(i) = strtod(path, &end);
                        path = end;
                    }
                    if (typ == 'M') current = value;
                    else current += value;
                    break;
                }
                
                if (!path_started) { path_start = current; path_started = true; }

                case 'L':
                case 'l':
                {
                    //Line to
                    Eigen::Vector2d value;
                    for (size_t i = 0; i < 2; i++)
                    {
                        while (*path < '0' || *path > '9') path++;
                        char *end;
                        value(i) = strtod(path, &end);
                        path = end;
                    }
                    Eigen::Vector2d next = (typ == 'L') ? (value) : (current + value);
                    _segments.push_back(new Line( std::array<Eigen::Vector2d, 2>{ current, next } ));
                    current = next;
                    break;
                }

                case 'H':
                case 'h':
                case 'V':
                case 'v':
                {
                    //Horizontal/vertical line to
                    double value;
                    while (*path < '0' || *path > '9') path++;
                    char *end;
                    value = strtod(path, &end);
                    path = end;
                    Eigen::Vector2d next = current;
                    if (typ == 'H') next(0) = value;
                    if (typ == 'h') next(0) += value;
                    if (typ == 'V') next(1) = value;
                    if (typ == 'v') next(1) += value;
                    _segments.push_back(new Line( std::array<Eigen::Vector2d, 2>{ current, next } ));
                    current = next;
                    break;
                }

                case 'Z':
                case 'z':
                {
                    //End of path
                    path++;
                    _segments.push_back(new Line( std::array<Eigen::Vector2d, 2>{ current, path_start } ));
                    current = path_start;
                    break;
                }

                case 'C':
                case 'c':
                {
                    //Cubic Bezier
                    Eigen::Vector2d points[3];
                    for (size_t j = 0; j < 3; j++)
                    {
                        for (size_t i = 0; i < 2; i++)
                        {
                            while (*path < '0' || *path > '9') path++;
                            char *end;
                            points[j](i) = strtod(path, &end);
                            path = end;
                        }
                        if (typ == 'c') points[j] += current;
                    }
                    _segments.push_back(new BezierCubic( std::array<Eigen::Vector2d, 4>{ current, points[0], points[1], points[2] } ));
                    previous_bezier_reflection = points[2] + (points[2] - points[1]);
                    current = points[2];
                    break;
                }

                case 'S':
                case 's':
                {
                    //Continue Cubic Bezier
                    Eigen::Vector2d points[3];
                    points[0] = (previous_typ == 'C' || previous_typ == 'c' || previous_typ == 'S' || previous_typ == 's') ? previous_bezier_reflection : current;
                    for (size_t j = 1; j < 3; j++)
                    {
                        for (size_t i = 0; i < 2; i++)
                        {
                            while (*path < '0' || *path > '9') path++;
                            char *end;
                            points[j](i) = strtod(path, &end);
                            path = end;
                        }
                        if (typ == 's') points[j] += current;
                    }
                    _segments.push_back(new BezierCubic( std::array<Eigen::Vector2d, 4>{ current, points[0], points[1], points[2] } ));
                    previous_bezier_reflection = points[2] + (points[2] - points[1]);
                    current = points[2];
                    break;
                }

                case 'Q':
                case 'q':
                {
                    //Quadratic Bezier
                    Eigen::Vector2d points[2];
                    for (size_t j = 0; j < 2; j++)
                    {
                        for (size_t i = 0; i < 2; i++)
                        {
                            while (*path < '0' || *path > '9') path++;
                            char *end;
                            points[j](i) = strtod(path, &end);
                            path = end;
                        }
                        if (typ == 'q') points[j] += current;
                    }
                    _segments.push_back(new BezierQuadratic( std::array<Eigen::Vector2d, 3>{ current, points[0], points[1] } ));
                    previous_bezier_reflection = points[0] + (points[1] - points[0]);
                    current = points[1];
                    break;
                }

                case 'T':
                case 't':
                {
                    //Continue quadratic Bezier
                    Eigen::Vector2d points[2];
                    points[0] = (previous_typ == 'Q' || previous_typ == 'q' || previous_typ == 'T' || previous_typ == 't') ? previous_bezier_reflection : current;
                    for (size_t i = 0; i < 2; i++)
                    {
                            while (*path < '0' || *path > '9') path++;
                            char *end;
                            points[1](i) = strtod(path, &end);
                            path = end;
                    }
                    if (typ == 't') points[1] += current;
                    _segments.push_back(new BezierQuadratic( std::array<Eigen::Vector2d, 3>{ current, points[0], points[1] } ));
                    previous_bezier_reflection = points[0] + (points[1] - points[0]);
                    current = points[1];
                    break;
                }

                case 'A':
                case 'a':
                {
                    //Arc
                    double values[7];
                    for (size_t i = 0; i < 7; i++)
                    {
                            while (*path < '0' || *path > '9') path++;
                            char *end;
                            values[i] = strtod(path, &end);
                            path = end;
                    }
                    Eigen::Vector2d next = (typ == 'A') ? (Eigen::Vector2d(values[5], values[6])) : (current + Eigen::Vector2d(values[5], values[6]));
                    Eigen::Vector2d radii = Eigen::Vector2d(values[0], values[1]);
                    _segments.push_back(new Arc( std::array<Eigen::Vector2d, 2>{ current, next }, radii, values[2], values[3] > 0.0, values[4] > 0.0 ));
                    current = next;
                    break;
                }

                case '\"': break;

                default: throw std::runtime_error("Drawing::Drawing: Invalid symbol");
            }
            previous_typ = typ;
        }
        path = strstr(path, "d=\"");
    }
}

size_t Drawing::segment_number() const
{
	return _segments.size();
}

const Segment &Drawing::segment(size_t number) const
{
	return *_segments[number];
}

double Drawing::width() const
{
	return _width;
}

double Drawing::height() const
{
	return _height;
}

Drawing::~Drawing()
{
	for (size_t i = 0; i < _segments.size(); i++) delete _segments[i];
}