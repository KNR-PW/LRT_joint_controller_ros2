#include "joint_controller_ros2_control/joint_controller_core.hpp"

using namespace joint_controller_core;

JointController::JointController(JointParameters _joint_params,
 pid_controller::PidParameters _pid_params, double _frequency): 
    joint_params_(_joint_params), pid_controller_(_pid_params, _frequency){}

double JointController::calculateEffort(const JointCommands& _joint_command,const JointStates& _joint_state)
{
    double position_error = _joint_command.desired_position_ - _joint_state.position_;
    double velocity_error = _joint_command.desired_velocity_ - _joint_state.velocity_;
    return pid_controller_.calculateEffort(position_error, velocity_error,
     _joint_command.feedforward_effort_, _joint_command.kp_scale_, _joint_command.kd_scale_);
}