#include "robot_joint_controller/robot_joint_controller.hpp"

#include <memory>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_joint_controller
{
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

RobotJointController::RobotJointController()
  : controller_interface::ControllerInterface(),
    command_subscriber_(nullptr),
    state_publisher_(nullptr)
{
  last_command_.q = 0.0;
  last_command_.dq = 0.0;
  last_command_.tau = 0.0;
  last_command_.kp = 0.0;
  last_command_.kd = 0.0;
  
  last_state_.q = 0.0;
  last_state_.dq = 0.0;
  last_state_.ddq = 0.0;
  last_state_.tau_est = 0.0;
  last_state_.cur = 0.0;
}

#if defined(ROS_DISTRO_HUMBLE)
CallbackReturn RobotJointController::on_init()
{
  return CallbackReturn::SUCCESS;
}
#endif

CallbackReturn RobotJointController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  name_space_ = get_node()->get_namespace();

  if (!get_node()->get_parameter("joints", joint_name_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joint given in namespace: '%s'", name_space_.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Configured joint: %s", joint_name_.c_str());

  // Start command subscriber
  command_subscriber_ = get_node()->create_subscription<robot_msgs::msg::MotorCommand>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotJointController::set_command_callback, this, std::placeholders::_1));

  // Start realtime state publisher
  state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<robot_msgs::msg::MotorState>>(
    get_node()->create_publisher<robot_msgs::msg::MotorState>("~/state", rclcpp::SystemDefaultsQoS()));

#if defined(ROS_DISTRO_FOXY)
  previous_update_timestamp_ = get_node()->get_clock()->now();
#endif

  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotJointController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.push_back(joint_name_ + "/" + HW_IF_EFFORT);
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration RobotJointController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.push_back(joint_name_ + "/" + HW_IF_POSITION);
  state_interfaces_config.names.push_back(joint_name_ + "/" + HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(joint_name_ + "/" + HW_IF_EFFORT);
  return state_interfaces_config;
}

CallbackReturn RobotJointController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize command with current position
  if (state_interfaces_.size() >= 1)
  {
    double init_pos = state_interfaces_[0].get_value();
    last_command_.q = init_pos;
    last_state_.q = init_pos;
  }
  
  last_command_.dq = 0.0;
  last_state_.dq = 0.0;
  last_command_.tau = 0.0;
  last_state_.tau_est = 0.0;

  // Reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<robot_msgs::msg::MotorCommand>();

  RCLCPP_INFO(get_node()->get_logger(), "Activated successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotJointController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<robot_msgs::msg::MotorCommand>();
  return CallbackReturn::SUCCESS;
}

#if defined(ROS_DISTRO_FOXY)
controller_interface::return_type RobotJointController::update()
{
  const auto current_time = get_node()->get_clock()->now();
  const auto period_seconds = (current_time - previous_update_timestamp_).seconds();
  previous_update_timestamp_ = current_time;
  
  auto joint_command = rt_command_ptr_.readFromRT();
  if (joint_command)
  {
    last_command_ = *joint_command;
  }
  
  update_controller(period_seconds);
  return controller_interface::return_type::OK;
}
#elif defined(ROS_DISTRO_HUMBLE)
controller_interface::return_type RobotJointController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto joint_command = rt_command_ptr_.readFromRT();
  if (joint_command)
  {
    last_command_ = *joint_command;
  }
  
  update_controller(period.seconds());
  return controller_interface::return_type::OK;
}
#endif

void RobotJointController::update_controller(double /*period_seconds*/)
{
  // Read current state from hardware interfaces
  if (state_interfaces_.size() >= 2)
  {
    last_state_.q = state_interfaces_[0].get_value();      // position
    last_state_.dq = state_interfaces_[1].get_value();     // velocity
    if (state_interfaces_.size() >= 3)
    {
      last_state_.tau_est = state_interfaces_[2].get_value();  // effort
    }
  }

  // Compute PD control + feedforward torque
  double pos_error = last_command_.q - last_state_.q;
  double vel_error = last_command_.dq - last_state_.dq;
  double commanded_torque = last_command_.kp * pos_error + 
                           last_command_.kd * vel_error + 
                           last_command_.tau;

  // Send torque command to hardware
  if (command_interfaces_.size() >= 1)
  {
    command_interfaces_[0].set_value(commanded_torque);
  }

  // Publish state
  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.q = last_state_.q;
    state_publisher_->msg_.dq = last_state_.dq;
    state_publisher_->msg_.ddq = last_state_.ddq;
    state_publisher_->msg_.tau_est = last_state_.tau_est;
    state_publisher_->msg_.cur = last_state_.cur;
    state_publisher_->unlockAndPublish();
  }
}

void RobotJointController::set_command_callback(const robot_msgs::msg::MotorCommand::SharedPtr msg)
{
  rt_command_ptr_.writeFromNonRT(*msg);
}

}  // namespace robot_joint_controller

PLUGINLIB_EXPORT_CLASS(
  robot_joint_controller::RobotJointController, controller_interface::ControllerInterface)

