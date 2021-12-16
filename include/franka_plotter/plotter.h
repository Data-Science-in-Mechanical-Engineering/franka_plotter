#pragma once
#include "drawing.h"
#include "configuration.h"
#include <Eigen/Geometry>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>

class Plotter
{
private:
    const Drawing *_drawing;
    Configuration _configuration;
    franka::Robot _robot;
    franka::Gripper _gripper;
    franka::Model _model;
    
    enum class State
    {
        plotting,
        ascending,
        moving,
        descending,
        preparing,
        returning,
        end
    };
    State _state            = State::moving;
    size_t _segment_time    = 0;
    size_t _segment         = 0;
    Eigen::Vector3d _initial_position;
    Eigen::Quaterniond _initial_orientation;

    double _compute_3d_scale() const;
    void _compute_3d_target(Eigen::Vector2d target, Eigen::Vector3d *position, Eigen::Quaterniond *orientation) const;
    void _compute_target(Eigen::Vector3d *position, Eigen::Quaterniond *orientation, Eigen::Vector3d *force);
    Eigen::Matrix<double, 7, 1> _compute_torques(const franka::RobotState &robot_state, Eigen::Vector3d target_position, Eigen::Quaterniond target_orientation, Eigen::Vector3d force) const;
    franka::Torques _control(const franka::RobotState &robot_state, franka::Duration time);

public:
    Plotter();
    void plot(const Drawing &drawing);
};