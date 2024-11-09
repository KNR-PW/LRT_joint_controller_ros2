#ifndef MOTEUS_CONTROLLER_INTERFACE_HPP_
#define MOTEUS_CONTROLLER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "ros2_moteus_position_controller/visibility_control.h"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

namespace moteus_controller_ros
{

  class MoteusControllerInterface : public controller_interface::ChainableControllerInterface
  {
  public:
    MOTEUS_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    MoteusControllerInterface();

    MOTEUS_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::CallbackReturn on_init() override;

    MOTEUS_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    MOTEUS_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    MOTEUS_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State & previous_state) override;

    MOTEUS_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    MOTEUS_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    MOTEUS_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    MOTEUS_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    MOTEUS_CONTROLLER_INTERFACE__VISIBILITY_PUBLIC 
    controller_interface::return_type update_and_write_commands(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

  protected:
    std::shared_ptr<position_controller::ParamListener> param_listener_; // TODO DODAJ TO
    position_controller::Params params_;

    std::vector<std::string> reference_and_state_dof_names_;
    size_t dof_;
    std::vector<double> measured_state_values_;

    using PidPtr = std::shared_ptr<control_toolbox::PidROS>;
    std::vector<PidPtr> pids_;
    // Feed-forward velocity weight factor when calculating closed loop pid adapter's command
    std::vector<double> feedforward_gain_;

    // Command subscribers and Controller State publisher
    rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

    rclcpp::Subscription<ControllerMeasuredStateMsg>::SharedPtr measured_state_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerMeasuredStateMsg>> measured_state_;

    rclcpp::Service<ControllerModeSrvType>::SharedPtr set_feedforward_control_service_;
    realtime_tools::RealtimeBuffer<feedforward_mode_type> control_mode_;

    using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

    rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
    std::unique_ptr<ControllerStatePublisher> state_publisher_;

    // override methods from ChainableControllerInterface
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

    bool on_set_chained_mode(bool chained_mode) override;

    // internal methods
    void update_parameters();
    controller_interface::CallbackReturn configure_parameters();

  private:
    // callback for topic interface
    PID_CONTROLLER__VISIBILITY_LOCAL
    void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  };

}  // namespace pid_controller

#endif  // PID_CONTROLLER__PID_CONTROLLER_HPP_