#pragma once
#include <Eigen/Dense>
#include <string>

class Configuration
{
private:
    std::string _ip = "192.168.0.1";
    double _speed = 0.03;
    double _length = 0.035;
    double _force = 0.01;
    std::array<Eigen::Vector3d, 3> _corners { Eigen::Vector3d(0.107581, -0.700828, -0.002), Eigen::Vector3d(-0.0912101, -0.693881, 0.0), Eigen::Vector3d(-0.0916694, -0.399395, 0.0) };
    void _set(const std::string param, const std::string value);

public:
    std::string ip() const;
    double speed() const;
    double length() const;
    double force() const;
    std::array<Eigen::Vector3d, 3> corners() const;
    Configuration();
};