#include "joint_controller_ros2_control/joint_controller_interface.hpp"

using namespace joint_controller_interface;

JointControllerInterface::JointControllerInterface(): controller_interface::ChainableControllerInterface(){}
controller_interface::CallbackReturn JointControllerInterface::on_init()
{
  try
  {
    param_listener_ = std::make_shared<joint_controller_parameters::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(get_node()->get_logger(),
           "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointControllerInterface::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(joint_num_);
    for (const auto & joint_name : joint_names_)
    {
        command_interfaces_config.names.push_back(joint_name + "/" + params_.command_interface);
    }
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration JointControllerInterface::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;

    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names.reserve(joint_num_ * params_.state_interfaces.size());
    for (const auto & interface : params_.state_interfaces)
    {
        for (const auto & joint_name: joint_names_)
        {
            state_interfaces_config.names.push_back(joint_name + "/" + interface);
        }
    }

    return state_interfaces_config;
}

controller_interface::CallbackReturn JointControllerInterface::on_cleanup(
      const rclcpp_lifecycle::State & previous_state)
{
    joint_names_.clear();
    joint_commands_.clear();
    joint_states_.clear();
    joint_controllers_.clear();
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointControllerInterface::on_configure(
      const rclcpp_lifecycle::State & previous_state)
{
    auto ret = configure_parameters();
    if (ret != CallbackReturn::SUCCESS)
    {
        return ret;
    }
}
controller_interface::CallbackReturn JointControllerInterface::on_activate(
      const rclcpp_lifecycle::State & previous_state)
{
    
}

controller_interface::CallbackReturn JointControllerInterface::on_deactivate(
      const rclcpp_lifecycle::State & previous_state){}
controller_interface::return_type JointControllerInterface::update_reference_from_subscribers(
      const rclcpp::Time & time, const rclcpp::Duration & period){}
controller_interface::return_type JointControllerInterface::update_and_write_commands(
      const rclcpp::Time & time, const rclcpp::Duration & period){}
bool JointControllerInterface::on_set_chained_mode(bool chained_mode){}

controller_interface::CallbackReturn JointControllerInterface::configure_parameters()
{
    if(params_.joint_names.size() != params_.pid_gains.joint_names_map.size())
    {
        RCLCPP_FATAL(get_node()->get_logger(),
            "Size of 'pid gains' (%zu) map and number or 'joint_names' (%zu) have to be the same!",
            params_.pid_gains.joint_names_map.size(), params_.joint_names.size());
        return CallbackReturn::FAILURE;
    }

    if(params_.joint_names.size() != params_.joint_params.joint_names_map.size())
    {
        RCLCPP_FATAL(get_node()->get_logger(),
            "Size of 'joint params' (%zu) map and number or 'joint_names' (%zu) have to be the same!",
            params_.joint_params.joint_names_map.size(), params_.joint_names.size());
        return CallbackReturn::FAILURE;
    }

    joint_num_ = arams_.joint_names.size();

    joint_commands_.resize(joint_num_);
    joint_states__.resize(joint_num_);

    double pid_frequency = params_.frequency;

    for(int i = 0; i < joint_num_; ++i)
    {
        joint_names_.push_back(params_.joint_names[i]);

        joint_controller_core::JointParameters joint_params;
        auto listener_joint_param = params_.joint_params.joint_names_map[params_.joint_names[i]];
        joint_params.position_max_ = listener_joint_param.position_max;
        joint_params.position_min_ = listener_joint_param.position_min;
        joint_params.position_offset_ = listener_joint_param.position_offset;
        joint_params.velocity_max_ = listener_joint_param.velocity_max;
        joint_params.effort_max_ = listener_joint_param.effort_max;


        pid_controller::PidParameters pid_params;
        auto listener_pid_param = params_.pid_gains.joint_names_map[params_.joint_names[i]];
        pid_params.proportional_coef_ = listener_pid_param.p;
        pid_params.derivative_coef_ = listener_pid_param.d;
        pid_params.integral_coef_ = listener_pid_param.i;
        pid_params.integration_limit_ = listener_pid_param.ilimit;

        joint_controller_core::JointController joint_controller(joint_params,
         pid_params, pid_frequency);
        
        joint_controllers_.push_back(joint_controller);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> JointControllerInterface::on_export_reference_interfaces()
{
}
std::vector<hardware_interface::StateInterface> JointControllerInterface::on_export_state_interfaces(){}