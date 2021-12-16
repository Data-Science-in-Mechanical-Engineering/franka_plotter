#include "../include/franka_plotter/plotter.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <algorithm>

void Plotter::_compute_3d_target(Eigen::Vector2d target, Eigen::Vector3d *position, Eigen::Quaterniond *orientation)
{
    std::array<Eigen::Vector3d, 3> corners = _configuration.corners();
    Eigen::Vector3d left = corners[1] - corners[0];
    Eigen::Vector3d down = corners[2] - corners[1];
    double width = (corners[1] - corners[0]).norm();
    double height = (corners[2] - corners[1]).norm();
    if (_drawing->width() / _drawing->height() > width / height)
    {
        //If drawing is widther
        *position = corners[0]
            + left * (target(0) / _drawing->width())
            + (width * ((target(1) - _drawing->height()/2) / _drawing->width()) + height/2) * (down/height);
    }
    else
    {
        //If drawing is taller
        *position = corners[0]
            + (height * ((target(0) - _drawing->width()/2) / _drawing->height()) + width/2) * (left/width)
            + down * (target(1) / _drawing->height());
    }
    *orientation = Eigen::AngleAxisd(atan2((*position)(1), (*position)(0)), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::Quaterniond(1,0,0,0);
}

void Plotter::_compute_target(Eigen::Vector3d *position, Eigen::Quaterniond *orientation, Eigen::Vector3d *force)
{
    const double maximal_ripple = 0.0001;
    const double height = 0.05;
    if (_state == State::plotting)
    {
        *force = Eigen::Vector3d(0, 0, -_configuration.force());
        double fraction = _segment_time * _configuration.speed() / _drawing->segment(_segment).length() / 1000;
        _compute_3d_target(_drawing->segment(_segment).point(std::min(fraction, 1.0)), position, orientation);
        
        if (fraction > 1.0)
        {
            if (_segment + 1 == _drawing->segment_number()) _state = State::ascending;
            else if ((_drawing->segment(_segment).point(1.0) - _drawing->segment(_segment + 1).point(0.0)).norm() > maximal_ripple) _state = State::ascending;
            else _segment++;
            _segment_time = 0;
        }
    }
    else if (_state == State::ascending)
    {
        *force = Eigen::Vector3d::Zero();
        _compute_3d_target(_drawing->segment(_segment).point(1.0), position, orientation);
        double fraction = _segment_time * _configuration.speed() / height / 1000;
        (*position)(2) += std::min(fraction, 1.0) * height;
        
        if (fraction > 1.0)
        {
            _state = (_segment + 1 == _drawing->segment_number()) ? State::returning : State::moving;
            _segment_time = 0;
        }
    }
    else if (_state == State::moving)
    {
        *force = Eigen::Vector3d::Zero();
        _compute_3d_target(_drawing->segment(_segment).point(1.0), position, orientation);
        Eigen::Vector3d next_position;
        Eigen::Quaterniond next_orientation;
        _compute_3d_target(_drawing->segment(_segment + 1).point(0.0), &next_position, &next_orientation);
        double distance = (*position - next_position).norm() + orientation->angularDistance(next_orientation);
        double fraction = _segment_time * _configuration.speed() / distance / 1000;
        *position = *position * std::max(1.0 - fraction, 0.0) + next_position * std::min(fraction, 1.0);
        (*position)(2) += height;
        *orientation = orientation->slerp(std::min(fraction, 1.0), next_orientation);

        if (fraction > 1.0)
        {
            _segment++;
            _state = State::descending;
            _segment_time = 0;
        }
    }
    else if (_state == State::descending)
    {
        *force = Eigen::Vector3d::Zero();
        _compute_3d_target(_drawing->segment(_segment).point(1.0), position, orientation);
        double fraction = _segment_time * _configuration.speed() / height / 1000;
        (*position)(2) += std::max(1.0 - fraction, 0.0) * height;
        if (fraction > 1.0)
        {
            _state = State::plotting;
            _segment_time = 0;
        }
    }
    else if (_state == State::preparing)
    {
        *force = Eigen::Vector3d::Zero();
        _compute_3d_target(_drawing->segment(0).point(0.0), position, orientation);
        double distance = (*position - _initial_position).norm() + orientation->angularDistance(_initial_orientation);
        double fraction = _segment_time * _configuration.speed() / distance / 1000;
        *position = std::max(1.0 - fraction, 0.0) * _initial_position + std::min(fraction, 1.0) * *position;
        (*position)(2) += height;
        *orientation = _initial_orientation.slerp(std::min(fraction, 1.0), *orientation);
        if (fraction > 1.0)
        {
            _state = State::descending;
            _segment_time = 0;
        }
    }
    else if (_state == State::returning)
    {
        *force = Eigen::Vector3d::Zero();
        _compute_3d_target(_drawing->segment(_drawing->segment_number() - 1).point(1.0), position, orientation);
        double distance = (*position - _initial_position).norm() + orientation->angularDistance(_initial_orientation);
        double fraction = _segment_time * _configuration.speed() / distance / 1000;
        *position = std::min(fraction, 1.0) * _initial_position + std::max(1.0 - fraction, 0.0) * *position;
        (*position)(2) += height;
        *orientation = orientation->slerp(std::min(fraction, 1.0), _initial_orientation);
        if (fraction > 1.0)
        {
            _state = State::end;
            _segment_time = 0;
        }
    }
}

Eigen::Matrix<double, 7, 1> Plotter::_compute_torques(const franka::RobotState &robot_state, Eigen::Vector3d target_position, Eigen::Quaterniond target_orientation, Eigen::Vector3d target_force)
{
    //Setting constants
    std::array<double, 16> stiffness_frame_array;
    Eigen::Affine3d stiffness_frame;
    stiffness_frame.translation()(2) = _configuration.length();
    Eigen::Matrix<double, 4, 4>::Map(&stiffness_frame_array[0]) = stiffness_frame.matrix();
    Eigen::Matrix<double, 6, 6> stiffness;
    stiffness.block<3,3>(0,0) = 100.0 * Eigen::Matrix<double, 3, 3>::Identity();
    stiffness.block<3,3>(3,3) = 10.0 * Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 6, 6> damping;
    damping.block<3,3>(0,0) = 2 * sqrt(100.0) * Eigen::Matrix<double, 3, 3>::Identity();
    damping.block<3,3>(3,3) = 2 * sqrt(10.0) * Eigen::Matrix<double, 3, 3>::Identity();

    //Getting current position
    Eigen::Affine3d transform(Eigen::Matrix<double, 4, 4>::Map(_model.pose(franka::Frame::kStiffness, robot_state.q, robot_state.F_T_EE, stiffness_frame_array).data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    Eigen::Matrix<double, 7, 1> joint_positions = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
    Eigen::Matrix<double, 7, 1> joint_velocities = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());
    Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Map(_model.zeroJacobian(franka::Frame::kStiffness, robot_state.q, robot_state.F_T_EE, stiffness_frame_array).data());
    Eigen::Matrix<double, 6, 1> velocity_rotation = jacobian * joint_velocities;
    Eigen::Matrix<double, 7, 1> coriolis = Eigen::Matrix<double, 7, 1>::Map(_model.coriolis(robot_state).data());

    //Computing torques
    Eigen::Matrix<double, 6, 1> error;
    error.segment<3>(0) = position - target_position;
    if (target_orientation.coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
    Eigen::Quaterniond error_quaternion(orientation.inverse() * target_orientation);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << -transform.linear() * error.tail(3);
    Eigen::Matrix<double, 7, 1> joint_torques = jacobian.transpose() * (- stiffness * error -damping * velocity_rotation) + coriolis;
    Eigen::Matrix<double, 6, 1> translation_rotation_load = Eigen::Matrix<double, 6, 1>::Zero();
    translation_rotation_load(2) = -_configuration.force();
    joint_torques += jacobian.transpose() * translation_rotation_load;

    return joint_torques;
}

franka::Torques Plotter::_control(const franka::RobotState &robot_state, franka::Duration time)
{
    _segment_time += time.toMSec();
    Eigen::Vector3d target_position;
    Eigen::Quaterniond target_orientation;
    Eigen::Vector3d target_force;
    franka::Torques torques{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    _compute_target(&target_position, &target_orientation, &target_force);
    Eigen::Matrix<double, 7, 1>::Map(&torques.tau_J[0]) = _compute_torques(robot_state, target_position, target_orientation, target_force);
    torques.motion_finished = (_state == State::end);
    return torques;
}

Plotter::Plotter() : _configuration(), _robot(_configuration.ip()), _model(_robot.loadModel())
{
    std::array<double, 16> stiffness_frame_array;
    Eigen::Affine3d stiffness_frame;
    stiffness_frame.translation()(2) = _configuration.length();
    Eigen::Matrix<double, 4, 4>::Map(&stiffness_frame_array[0]) = stiffness_frame.matrix();
    Eigen::Affine3d transform(Eigen::Matrix<double, 4, 4>::Map(_model.pose(franka::Frame::kStiffness, _robot.readOnce().q, _robot.readOnce().F_T_EE, stiffness_frame_array).data()));
    _initial_position = transform.translation();
    _initial_orientation = transform.linear();
    
    /*
    //Getting executable's directory
    std::string directory(64, '\0');
    while (true)
    {
        ssize_t size = readlink("/proc/self/exe", &directory[0], directory.size());
        if (size < 0) throw std::runtime_error("Plotter::Plotter: Could not find executable directory");
        else if (size == directory.size()) directory.resize(2 * directory.size(), '\0');
        else break;
    }
    while (directory.back() != '/') directory.pop_back();

    //Loading model
    struct stat model_stat;
    if (stat((directory + "model/model.urdf").c_str(), &model_stat) == 0) pinocchio::urdf::buildModel(directory + "model/model.urdf", _model);
    if (stat((directory + "../model/model.urdf").c_str(), &model_stat) == 0) pinocchio::urdf::buildModel(directory + "../model/model.urdf", _model);
    else throw std::runtime_error("Plotter::Plotter: Could not find model file");
    _data = pinocchio::Data(_model);
    */
}

void Plotter::plot(const Drawing &drawing)
{
    _drawing = &drawing;
    _segment = 0;
    _segment_time = 0;
    _state = State::preparing;

    Plotter *plotter = this;
    _robot.control([plotter](const franka::RobotState &robot_state, franka::Duration time) -> franka::Torques
    {
        return plotter->_control(robot_state, time);
    });
}