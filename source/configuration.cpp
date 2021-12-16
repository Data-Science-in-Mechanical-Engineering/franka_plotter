#include "../include/franka_plotter/configuration.h"
#include <fstream>

void Configuration::_set(const std::string param, const std::string value)
{
    if (param == "IP")
    {
        _ip = value;
    }
    else
    {
        char *end;
        double dvalue = strtod(value.data(), &end);
        if (*end != '\0') throw std::runtime_error("Configuration::_set: Invalid value");
        else if (param == "SPEED") _speed = dvalue;
        else if (param == "LENGTH") _length = dvalue;
        else if (param == "FORCE") _force = dvalue;
        else if (param == "CORNER_1_X") _corners[0](0) = dvalue;
        else if (param == "CORNER_1_Y") _corners[0](1) = dvalue;
        else if (param == "CORNER_1_Z") _corners[0](2) = dvalue;
        else if (param == "CORNER_2_X") _corners[1](0) = dvalue;
        else if (param == "CORNER_2_Y") _corners[1](1) = dvalue;
        else if (param == "CORNER_2_Z") _corners[1](2) = dvalue;
        else if (param == "CORNER_3_X") _corners[2](0) = dvalue;
        else if (param == "CORNER_3_Y") _corners[2](1) = dvalue;
        else if (param == "CORNER_3_Z") _corners[2](2) = dvalue;
        else throw std::runtime_error("Configuration::_set: Invalid parameter");
    }
}

std::string Configuration::ip() const
{
    return _ip;
}

double Configuration::speed() const
{
    return _speed;
}

double Configuration::length() const
{
    return _length;
}

double Configuration::force() const
{
    return _force;
}

std::array<Eigen::Vector3d, 3> Configuration::corners() const
{
    return _corners;
}

Configuration::Configuration()
{
    enum class State
    {
        wait_param_name_begin,
        wait_param_name_end,
        wait_equal,
        wait_value_begin,
        wait_value_end,
        wait_line_end,
        wait_comment_end
    };

    std::ifstream file("franka_plotter.config", std::ios::binary);
    if (!file.good()) return;

    std::string param;
    std::string value;
    State state = State::wait_param_name_begin;
    while (true)
    {
        char c;
        bool eof = !file.read(&c, 1);
        if (!eof & (c < 0x20 || c >= 0x7F )) throw std::runtime_error("Configuration::Configuration: Invalid symbol");

        switch (state)
        {
        case State::wait_param_name_begin:
            if (eof) return;
            else if (c == '\r' || c == '\n') {}
            else if (c == ' ' || c == '\t') {}
            else if (c == '#') state = State::wait_line_end;
            else if (c == '=') throw std::runtime_error("Configuration::Configuration: Unexpected equality sign");
            else { param = std::string(1, c); state = State::wait_param_name_end; }
            break;


        case State::wait_param_name_end:
            if (eof) throw std::runtime_error("Configuration::Configuration: Unexpected end of file");
            else if (c == '\r' || c == '\n') throw std::runtime_error("Configuration::Configuration: Unexpected new line");
            else if (c == ' ' || c == '\t') state = State::wait_equal;
            else if (c == '#') throw std::runtime_error("Configuration::Configuration: Unexpected comment");
            else if (c == '=') state = State::wait_value_begin;
            else param.push_back(c);
            break;

        case State::wait_equal:
            if (eof) throw std::runtime_error("Configuration::Configuration: Unexpected end of file");
            else if (c == '\r' || c == '\n') throw std::runtime_error("Configuration::Configuration: Unexpected new line");
            else if (c == ' ' || c == '\t') {}
            else if (c == '#') throw std::runtime_error("Configuration::Configuration: Unexpected comment");
            else if (c == '=') state = State::wait_value_begin;
            else throw std::runtime_error("Configuration::Configuration: Unexpected character");

        case State::wait_value_begin:
            if (eof) throw std::runtime_error("Configuration::Configuration: Unexpected end of file");
            else if (c == '\r' || c == '\n') throw std::runtime_error("Configuration::Configuration: Unexpected new line");
            else if (c == ' ' || c == '\t') {}
            else if (c == '#') throw std::runtime_error("Configuration::Configuration: Unexpected comment");
            else if (c == '=') throw std::runtime_error("Configuration::Configuration: Unexpected equality sign");
            else { value = std::string(1, c); state = State::wait_value_end; }
            break;

        case State::wait_value_end:
            if (eof) { _set(param, value); return; }
            else if (c == '\r' || c == '\n') { _set(param, value); state = State::wait_param_name_begin; }
            else if (c == ' ' || c == '\t') { _set(param, value); state = State::wait_line_end; }
            else if (c == '#') { _set(param, value); state = State::wait_comment_end; }
            else if (c == '=') throw std::runtime_error("Configuration::Configuration: Unexpected equality sign");
            else value.push_back(c);
            break;

        case State::wait_line_end:
            if (eof) return;
            else if (c == '\r' || c == '\n') state = State::wait_param_name_begin;
            else if (c == ' ' || c == '\t') {}
            else if (c == '#') state = State::wait_comment_end;
            else if (c == '=') throw std::runtime_error("Configuration::Configuration: Unexpected equality sign");
            else throw std::runtime_error("Configuration::Configuration: Unexpected character");
            break;

        case State::wait_comment_end:
            if (eof) return;
            else if (c == '\r' || c == '\n') state = State::wait_param_name_begin;
            else if (c == ' ' || c == '\t') {}
            else if (c == '#') {}
            else if (c == '=') {}
            else {}
            break;
        }
    }
}